# SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

'''
Steps to run these cases:
- Build
  - . ${IDF_PATH}/export.sh
  - pip install idf_build_apps
  - python tools/build_apps.py components/openai/test_apps -t esp32s2
- Test
  - pip install -r tools/requirements/requirement.pytest.txt
  - pytest components/openai/test_apps --target esp32s3
'''

import pytest
from pytest_embedded import Dut

@pytest.mark.target('esp32s2')
@pytest.mark.target('esp32s3')
@pytest.mark.env('generic')
@pytest.mark.parametrize(
    'config',
    [
        'defaults',
    ],
)
def test_openai(dut: Dut)-> None:
    dut.expect_exact('Press ENTER to see the list of tests.')
    dut.write('*')
    dut.expect_unity_test_output(timeout = 1000)
