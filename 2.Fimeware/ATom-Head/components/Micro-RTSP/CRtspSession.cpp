#include "CRtspSession.h"
#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>

CRtspSession::CRtspSession(SOCKET aRtspClient, CStreamer * aStreamer) : m_RtspClient(aRtspClient),m_Streamer(aStreamer)
{
    DEBUG_PRINT("Creating RTSP session\n");
    newCommandInit();

    m_RtspSessionID  = getRandom();         // create a session ID
    //m_RtspSessionID |= 0x80000000; //TODO why? just forces the session to be negative, which isn't spec
    m_StreamID       = -1;
    m_ClientRTPPort  =  0;
    m_ClientRTCPPort =  0;
    m_TcpTransport   =  false;
    m_streaming = false;
    m_stopped = false;

    m_CSeq = 0; // CSeq sequense must be kept through the whole session
    m_RtspCmdType = RTSP_UNKNOWN;

    RecvBuf = new char[RTSP_BUFFER_SIZE];
    Response = new char[RESPONSE_BUFFER_SIZE];
    SDPBuf = new char[OBUF_SIZE];
    OBuf = new char[OBUF_SIZE];
    Transport = new char[TRANSPORT_BUFFER_SIZE];
    timeBuf = new char[RTSP_PARAM_STRING_MAX];
    CmdName = new char[20];
}

CRtspSession::~CRtspSession()
{
    tcpsocketclose(m_RtspClient);

    delete[] RecvBuf;
    delete[] Response;
    delete[] SDPBuf;
    delete[] OBuf;
    delete[] Transport;
    delete[] timeBuf;
    delete[] CmdName;
}

/*! @brief Initialize stuff for processing new client's command */
void CRtspSession::newCommandInit()
{
    memset( m_CommandPresentationPart, 0x00, sizeof( m_CommandPresentationPart ) );
    memset( m_CommandStreamPart,    0x00, sizeof( m_CommandStreamPart ) );
    memset( m_CommandHostPort,  0x00, sizeof( m_CommandHostPort ) );
    m_ContentLength = 0;
}

/*! @brief read numeric stuff after header name and check all possible sanity
    @param buf source buffer
    @param number the number
    @param max_length length of buf
    @return NULL if error or pointer to the rest of line
*/
static char * parse_numeric_header( char *buf, unsigned int *number, int max_length )
{
    int count = max_length;

    while ( *buf  && count > 0 && ( *buf == ' ' || *buf == '\t' ) ) // skipping space after ':'
    {
        ++buf;
        --count;
    }

    if ( ! *buf || ! isdigit( *buf ) || ! count )
        return NULL;

    char *number_start = buf;

    while( *buf && isdigit( *buf ) && count > 0 )
    {
        ++buf;
        --count;
    }

    if ( count == 0 )
        return NULL;

    char c = *buf;

    *buf = '\0';
    *number = atoi( number_start );
    *buf = c;

    return buf;
}

/*! @brief Called internally to fully parse new command from the client */
bool CRtspSession::ParseRtspRequest( char * aRequest, unsigned aRequestSize )
{
    newCommandInit();

    /* now our typical command will be like:
    [CRLF]
    SETUP rtsp://server.example.com/mjpeg/1 RTSP/1.0
    CSeq: 2
    Transport: RTP/AVP;unicast;something;
        client_port=7000-7001;somethingelse
    CRLF
    but we will use a required subset from rfc2326 as per Table 2 (https://tools.ietf.org/html/rfc2326#section-10)
    */

    char *cur_pos = aRequest;
    int dst_pos = 0; // will reuse this to copy some parts into internal variables

    // 1st doing basic sanity check and URI parsing
    while ( dst_pos < 19 && *cur_pos != ' ' && *cur_pos != '\t' ) // skip possible CRLF and command name as we alredy got it in the handleRequests()
    {
        CmdName[ dst_pos++ ] = *(cur_pos++);
    }

    CmdName[ dst_pos ] = '\0';

    while ( *cur_pos && isspace( *cur_pos ) )
        ++cur_pos;

    if ( ! *cur_pos || 0 != strncasecmp( "rtsp://", cur_pos, 7 ) )
        return false;

    cur_pos += 7;

    // getting host:port
    for( dst_pos = 0; *cur_pos && ! isspace ( *cur_pos ) && *cur_pos != '/'; ++cur_pos, ++dst_pos )
    {
        if ( dst_pos == MAX_HOSTNAME_LEN )
            return false;

        m_CommandHostPort[ dst_pos ] = *cur_pos;
    }

    if ( *cur_pos != '/' ) // no next part
        return false;

    m_CommandHostPort[ dst_pos ] = '\0';
    DEBUG_PRINT( "host-port: %s\n", m_CommandHostPort );

    while ( *cur_pos == '/' )
        ++cur_pos;

    // getting presentation part
    for( dst_pos = 0; *cur_pos && ! isspace ( *cur_pos ) && *cur_pos != '/'; ++cur_pos, ++dst_pos )
    {
        if ( dst_pos == RTSP_PARAM_STRING_MAX )
            return false;

        m_CommandPresentationPart[ dst_pos ] = *cur_pos;
    }

    if ( *cur_pos != '/' ) // no next part
        return false;

    m_CommandPresentationPart[ dst_pos ] = '\0';
    DEBUG_PRINT( "+ pres: %s\n", m_CommandPresentationPart );

    while ( *cur_pos == '/' )
        ++cur_pos;

    // getting stream part
    for( dst_pos = 0; *cur_pos && ! isspace ( *cur_pos ) && *cur_pos != '/'; ++cur_pos, ++dst_pos )
    {
        if ( dst_pos == RTSP_PARAM_STRING_MAX )
            return false;

        m_CommandStreamPart[ dst_pos ] = *cur_pos;
    }

    m_CommandStreamPart[ dst_pos ] = '\0';

    while ( *cur_pos == '/' ) // vlc sometimes put extra / after session name on setup 
        ++cur_pos;

    if ( *cur_pos != ' ' && *cur_pos != '\t' ) // no final RTSP/x.x
        return false;

    DEBUG_PRINT( "+ stream: %s\n", m_CommandStreamPart );

    while ( isspace( *cur_pos ) )
        ++cur_pos;

    if ( 0 != strncmp( "RTSP/", cur_pos, 5 ) )
        return false;

    cur_pos += 5;
    if ( ! isdigit( *cur_pos ) || cur_pos[ 1 ] != '.' || ! isdigit( cur_pos[ 2 ] ) )
        return false;

    cur_pos += 3;

    // now looping through header lines and picking up what matter to us
    int left; // rough estimate of buffer space left to examine.
    // note that initial reader already put \0 mark in the buffer somewhere, so we only need to carefully check for it
    DEBUG_PRINT( "### analyzing headers\n" );
    
    for(;;)
    {
        // skipping leftovers from previous line
        while ( *cur_pos && *cur_pos != '\r' && cur_pos[ 1 ] != '\n' )
            ++cur_pos;

        // at the end of headers block there must be CR,LF,CR,LF always, then either the body or \0
        if ( ! *cur_pos || ( *cur_pos != '\r' && cur_pos[ 1 ] != '\n' ) ) // still some unexpected garbage?
            return false;

        cur_pos += 2; // skip CRLF

        if ( ! *cur_pos ) // we're done with headers
            break;

        left = aRequestSize - ( cur_pos - aRequest );

        // we're at the begin of the next header line now
        #ifdef ENABLE_DEBUG // a little window to our current line beginning
            printf( "* left: %d: '", left );
            for( char *s = cur_pos; *s && (s - cur_pos) < 20; ++s)
                if( *s == '\r' )
                    printf ( "<CR>" );
                else if( *s == '\n' )
                    printf ( "<LF>" );
                else if ( isprint( *s ) )
                    putchar( *s );
                else
                    printf( "<0x%x>", *s );
            puts( "'" );
        #endif

        // now we're at the start of another header's line

        if ( 0 == strncmp( "CSeq:", cur_pos, 5) )
        {
            unsigned new_cseq;

            left -= 5;
            cur_pos = parse_numeric_header( cur_pos + 5, &new_cseq, left );

            if( cur_pos == NULL )
                return false;

            m_CSeq = new_cseq; // we may check something here or later maybe...

            DEBUG_PRINT( "+ got cseq: %u\n", new_cseq );

            continue; // loop to next line
        }

        if ( 0 == strncmp( "Content-Length:", cur_pos, 15) )
        {
            left -= 15;
            cur_pos = parse_numeric_header( cur_pos + 15, &m_ContentLength, left );

            if( cur_pos == NULL )
                return false;

            DEBUG_PRINT( "+ got cont-len: %u\n", m_ContentLength );

            continue; // loop to next line
        }

        // for other headers we gluing continued strings together to simplify analysis
        for ( char *p = cur_pos; *p; ++p )
        {
            // peeking past CRLF: if there is space - it is continued header line
            if ( *p == '\r' && p[ 1 ] == '\n' )
            {
                if ( p[ 2 ] != ' ' && p[ 2 ] != '\t' ) // no space. ending search
                    break;

                // clearing and looking for another continuation
                *p = ' ';
                ++p;
                *p = ' ';
            }
        }

        // transport settings: proto, ports, etc
        if ( m_RtspCmdType == RTSP_SETUP && 0 == strncmp( "Transport:", cur_pos, 10 ) )
        {
            cur_pos += 10;
            while( *cur_pos && isspace( *cur_pos ) )
                ++cur_pos;
            
            if ( 0 != strncmp( cur_pos, "RTP/AVP", 7) ) // std says this is mandatory part
                return false;
            
            cur_pos += 7;

            if ( 0 == strncmp( cur_pos, "/TCP", 4 ) ) // TCP is also good?
            {
                m_TcpTransport = true;
                cur_pos += 4;
            }
            else
                m_TcpTransport = false;

            DEBUG_PRINT( "+ Transport is %s\n", (m_TcpTransport ? "TCP" : "UDP") );

            m_ClientRTPPort = 0;

            // now looking for sub-params like clent_port=
            char *next_part, last_char;
            for(;;)
            {
                while( *cur_pos == ';' || *cur_pos == ' ' || *cur_pos == '\t' )
                    ++cur_pos;

                if ( ! *cur_pos )
                    return false;

                if ( *cur_pos == '\r' && cur_pos[ 1 ] == '\n' )
                    break;

                next_part = strpbrk( cur_pos, ";\r" ); // gettin pointer to the next sub-param if any
                if( ! next_part )
                    return false;

                last_char = *next_part; // in case we'll need to put \0 here

                if ( 0 == strncmp( cur_pos, "client_port=", 12 ) ) // "client_port" "=" port [ "-" port ]
                {
                    char *p = ( cur_pos += 12 );
                    while( isdigit( *p ) )
                        ++p;

                    if ( p == cur_pos )
                        return false;

                    *p = '\0';

                    m_ClientRTPPort  = atoi( cur_pos );
                    m_ClientRTCPPort = m_ClientRTPPort + 1;
                    DEBUG_PRINT( "+ got client port: %u\n", m_ClientRTPPort );
                }

                *next_part = last_char; // restoring if changed

                cur_pos = next_part;
            }
        } // Transport:

        if ( *cur_pos != '\r' )
        {
            ERROR_PRINT( "? unknown header ?\n" );
        }

        // ignored headers are skipped. we left current position at the CRLF so next loop is going smoothly
        while ( *cur_pos && *cur_pos != '\r' )
            ++cur_pos;
    } // loop though headers

    DEBUG_PRINT( "\n+ RTSP command: %s\n", CmdName );

    return true;
}

RTSP_CMD_TYPES CRtspSession::Handle_RtspRequest( char *aRequest, unsigned aRequestSize )
{
    if ( ParseRtspRequest( aRequest, aRequestSize ) )
    {
        switch ( m_RtspCmdType )
        {
            case RTSP_OPTIONS:  Handle_RtspOPTION();   break;
            case RTSP_DESCRIBE: Handle_RtspDESCRIBE(); break;
            case RTSP_SETUP:    Handle_RtspSETUP();    break;
            case RTSP_PLAY:     Handle_RtspPLAY();     break;
            default: break;
        }
    }

    return m_RtspCmdType;
}

void CRtspSession::Handle_RtspOPTION()
{
    snprintf(Response, RESPONSE_BUFFER_SIZE,
             "RTSP/1.0 200 OK\r\nCSeq: %u\r\n"
             "Public: DESCRIBE, SETUP, TEARDOWN, PLAY, PAUSE\r\n\r\n", m_CSeq);

    tcpsocketsend(m_RtspClient,Response,strlen(Response));
}

void CRtspSession::Handle_RtspDESCRIBE()
{
    // check whether we know a stream with the URL which is requested
    m_StreamID = -1;        // invalid URL

    if ( m_Streamer->getURIPresentation() == m_CommandPresentationPart &&
            m_Streamer->getURIStream() == m_CommandStreamPart )
        m_StreamID = 0;

    if ( m_StreamID == -1 )
    {   // Stream not available
        snprintf( Response, RESPONSE_BUFFER_SIZE,
                 "RTSP/1.0 404 Stream Not Found\r\nCSeq: %u\r\n%s\r\n",
                 m_CSeq,
                 DateHeader());

        tcpsocketsend( m_RtspClient, Response, strlen(Response) );
        return;
    }

    // simulate DESCRIBE server response
    //char * ColonPtr;
    //strcpy( OBuf, m_CommandHostPort );
    //ColonPtr = strstr( OBuf, ":" );
    //if (ColonPtr != nullptr) ColonPtr[0] = 0x00; //TODO good?

    snprintf( SDPBuf, RESPONSE_BUFFER_SIZE,
             "v=0\r\n"
             "o=- %d %d IN IP4 %s\r\n"
             "s=\r\n"
             "t=0 0\r\n"                                       // start / stop - 0 -> unbounded and permanent session
             "m=video 0 RTP/AVP 26\r\n"
             "c=IN IP4 0.0.0.0\r\n",
             (int)getRandom(),
             (int)getRandom(),
             m_CommandHostPort );

    snprintf( Response, RESPONSE_BUFFER_SIZE,
             "RTSP/1.0 200 OK\r\nCSeq: %u\r\n"
             "%s\r\n"
             "Content-Base: rtsp://%s/%s/%s/\r\n"
             "Content-Type: application/sdp\r\n"
             "Content-Length: %d\r\n\r\n"
             "%s",
             m_CSeq,
             DateHeader(),
             m_CommandHostPort, m_CommandPresentationPart, m_CommandStreamPart,
             (int) strlen(SDPBuf),
             SDPBuf);

    tcpsocketsend( m_RtspClient, Response, strlen(Response) );
}

void CRtspSession::Handle_RtspSETUP()
{
    static char Response[1024];
    static char Transport[255];

    // init RTP streamer transport type (UDP or TCP) and ports for UDP transport
    m_Streamer->InitTransport(m_ClientRTPPort,m_ClientRTCPPort,m_TcpTransport);

    // simulate SETUP server response
    if (m_TcpTransport)
        snprintf(Transport,TRANSPORT_BUFFER_SIZE,"RTP/AVP/TCP;unicast;interleaved=0-1");
    else
        snprintf(Transport,TRANSPORT_BUFFER_SIZE,
                 "RTP/AVP;unicast;destination=127.0.0.1;source=127.0.0.1;client_port=%i-%i;server_port=%i-%i", //TODO set IP addresses
                 m_ClientRTPPort,
                 m_ClientRTCPPort,
                 m_Streamer->GetRtpServerPort(),
                 m_Streamer->GetRtcpServerPort());
    snprintf(Response,RESPONSE_BUFFER_SIZE,
             "RTSP/1.0 200 OK\r\nCSeq: %u\r\n"
             "%s\r\n"
             "Transport: %s\r\n"
             "Session: %i\r\n\r\n",
             m_CSeq,
             DateHeader(),
             Transport,
             m_RtspSessionID);

    tcpsocketsend(m_RtspClient,Response,strlen(Response));
}

void CRtspSession::Handle_RtspPLAY()
{
    // simulate SETUP server response
    snprintf(Response, RESPONSE_BUFFER_SIZE,
             "RTSP/1.0 200 OK\r\nCSeq: %u\r\n"
             "%s\r\n"
             "Range: npt=0.000-\r\n"
             "Session: %i\r\n"
             "RTP-Info: url=rtsp://%s/%s/%s/\r\n\r\n", // FIXME
             m_CSeq,
             DateHeader(),
             m_RtspSessionID,
             m_CommandHostPort, m_CommandPresentationPart, m_CommandStreamPart);

    tcpsocketsend(m_RtspClient,Response,strlen(Response));
}

char const * CRtspSession::DateHeader()
{
    time_t tt = time(NULL);
    strftime(timeBuf, RTSP_PARAM_STRING_MAX, "Date: %a, %b %d %Y %H:%M:%S GMT", gmtime(&tt));
    return timeBuf;
}

int CRtspSession::GetStreamID()
{
    return m_StreamID;
};

/**
   Read from our socket, parsing commands as possible.
 */
bool CRtspSession::handleRequests( uint32_t readTimeoutMs )
{
    if ( m_stopped )
        return false; // Already closed down

    bufPos = 0; // current position into receiving buffer. used to glue split requests.
    state = hdrStateUnknown;

    if ( bufPos == 0 || bufPos >= RTSP_BUFFER_SIZE - 1 ) // in case of bad client
    {
        memset( RecvBuf, 0x00, RTSP_BUFFER_SIZE );
        bufPos = 0;
        state = hdrStateUnknown;
    }

    // we always read 1 byte less than the buffer length, so all string ops here will not panic
    int res = tcpsocketread( m_RtspClient, RecvBuf + bufPos, RTSP_BUFFER_SIZE - bufPos - 1, readTimeoutMs );
    if ( res > 0 )
    {
        bufPos += res;
        RecvBuf[ bufPos ] = '\0';

        DEBUG_PRINT( "+ read %d bytes\n", res );

        if ( state == hdrStateUnknown && bufPos >= 6 ) // we need at least 4-letter at the line start with optional heading CRLF
        {
            if( NULL != strstr( RecvBuf, "\r\n" ) ) // got a full line
            {
                char *s = RecvBuf;
                if ( *s == '\r' && *(s + 1) == '\n' ) // skip allowed empty line at front
                    s += 2;

                newCommandInit();
                // find out the command type
                m_RtspCmdType = RTSP_UNKNOWN;

                if ( strncmp( s, "OPTIONS ", 8 ) == 0 )        m_RtspCmdType = RTSP_OPTIONS;
                else if ( strncmp( s, "DESCRIBE ", 9 )  == 0 ) m_RtspCmdType = RTSP_DESCRIBE;
                else if ( strncmp( s, "SETUP ", 6 )     == 0 ) m_RtspCmdType = RTSP_SETUP;
                else if ( strncmp( s, "PLAY ", 5 )      == 0 ) m_RtspCmdType = RTSP_PLAY;
                else if ( strncmp( s, "TEARDOWN ", 9 )  == 0 ) m_RtspCmdType = RTSP_TEARDOWN;

                if( m_RtspCmdType != RTSP_UNKNOWN ) // got some
                    state = hdrStateGotMethod;
                else
                    state = hdrStateInvalid;
            }
        } // if state == hdrStateUnknown

        if ( state != hdrStateUnknown ) // in all cases we need to slurp the whole header before answering
        {
            // per https://tools.ietf.org/html/rfc2326 we need to look for an empty line
            // to be sure that we got the correctly formed header. Also starting CRLF should be ignored.
            char *s = strstr( bufPos > 4 ? RecvBuf + bufPos - 4 : RecvBuf, "\r\n\r\n" ); // try to save cycles by searching in the new data only
            
            if ( s == NULL ) // no end of header seen yet
                return true;

            if ( state == hdrStateInvalid ) // tossing some immediate answer, so client don't fall into endless stupor
            {
                // not sure which code is more appropriate and if CSeq is needed here?
                int l = snprintf( RecvBuf, RTSP_BUFFER_SIZE, "RTSP/1.0 400 Bad Request\r\nCSeq: %u\r\n\r\n", m_CSeq );
                tcpsocketsend( m_RtspClient, RecvBuf, l );
                bufPos = 0;
                return false;
            }
        }

        RTSP_CMD_TYPES C = Handle_RtspRequest( RecvBuf, res );

        if ( C == RTSP_PLAY )
            m_streaming = true;

        else if ( C == RTSP_TEARDOWN )
        {
            m_stopped = true;
        }

        // cleaning up
        state = hdrStateUnknown;
        bufPos = 0;

        return true;
    } // res > 0
    else if ( res == 0 )
    {
        DEBUG_PRINT("client closed socket, exiting\n");
        m_stopped = true;
        return true;
    }
    else
    {
        // Timeout on read
        return false;
    }
}

void CRtspSession::broadcastCurrentFrame(uint32_t curMsec) {
    // Send a frame
    if (m_streaming && !m_stopped) {
        DEBUG_PRINT("serving a frame\n");
        m_Streamer->streamImage(curMsec);
    }
}
