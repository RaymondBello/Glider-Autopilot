import time
from nav import NavigationComputer

# Declare test class
nav = NavigationComputer()

# Read Write delay in seconds
rw_delay = 0.0


def test_fc_modes():

    print(f"Test started @ {time.strftime('%H:%M:%S', time.localtime())}")
    
    assert nav.ERROR == 0, f'Mode:ERROR should be 0. Actual={nav.ERROR}'
    assert nav.UNINITIALIZED == 1, f'Mode:UNINITIALIZED should be 1. Actual={nav.UNINITIALIZED}'
    assert nav.INITIALIZED == 2, f'Mode:UNINITIALIZED should be 2. Actual={nav.INITIALIZED}'
    assert nav.ACTIVE == 4, f'Mode:ACTIVE should be 4. Actual={nav.ACTIVE}'
    assert nav.IDLE == 8, f'Mode:IDLE should be 8. Actual={nav.IDLE}'
    assert nav.REBOOT == 16, f'Mode:REBOOT should be 16. Actual={nav.REBOOT}'
    
nav.serial.connect('COM7')

def test_fc_mode_transitions():
    
    print(f"Test started @ {time.strftime('%H:%M:%S', time.localtime())}")
    
    nav.fc_set_mode(nav.UNINITIALIZED)
    serial_str = str(nav.serial.getData())
    print(serial_str)
    
    time.sleep(rw_delay)
    
    nav.fc_get_mode()
    time.sleep(rw_delay)
    serial_str = str(nav.serial.getData())
    print(serial_str)
    
    # assert ('INITIALIZATION' in serial_str), f"Received invalid confirmation. Actual={serial_str}"
    
    
    

