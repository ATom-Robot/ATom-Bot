# Install script for directory: D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/ATom-Head")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/softwoare/tools_dept/Espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-objdump.exe")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/aesni.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/arc4.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/blowfish.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/bn_mul.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/certs.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/cipher_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/compat-1.3.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/config.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ecp_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/entropy_poll.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/havege.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/md2.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/md4.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/md_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/net.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/padlock.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pk_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pkcs11.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/rsa_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl_internal.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/mbedtls/xtea.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "D:/softwoare/tools_dept/Espressif/frameworks/esp-idf-v4.4.2/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

