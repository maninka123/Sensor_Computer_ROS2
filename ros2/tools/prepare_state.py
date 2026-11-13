#!/usr/bin/env python3
"""Create fresh-clone runtime state without overwriting existing settings."""
from pathlib import Path
import os

root=Path(__file__).resolve().parents[1]
if os.getuid()==0:
    raise SystemExit('Prepare/build as the deployment user, not root.')
home=root/'state/home'
privacy=home/'.config/spinnaker/privacy.xml'
privacy.parent.mkdir(parents=True,exist_ok=True)
if not os.access(home,os.W_OK):
    raise SystemExit('Runtime state is not writable by the deployment user.')
if not privacy.exists():
    privacy.write_text('<PrivacyConfig><Disabled>true</Disabled></PrivacyConfig>\n')
print('Runtime state is ready.')
