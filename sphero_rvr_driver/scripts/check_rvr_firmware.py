#!/usr/bin/env python3

"""
This script checks the firmware version of the RVR to periodically check for updates.
"""

import time
from sphero_sdk import SpheroRvrObserver


def main():
    """Main function to run the RVR firmware check.
    """
    rvr = SpheroRvrObserver()
    rvr._write_timestamp()  # hack to delay the firware check


# main
if __name__ == '__main__':
    main()
