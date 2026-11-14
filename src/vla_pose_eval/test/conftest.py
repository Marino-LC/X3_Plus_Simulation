import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vla_pose_eval.config import load_config  # noqa: E402


@pytest.fixture(scope='session')
def cfg():
    return load_config(os.path.join(os.path.dirname(__file__), '..', 'config'))
