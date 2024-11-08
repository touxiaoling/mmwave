import time
import mmwcas

config_dict ={}
record_duration = 1*60
status = mmwcas.mmw_set_config(config_dict)
if status!=0:
    print(status)
    raise ValueError(f"{status}")

status = mmwcas.mmw_init()
assert status==0,ValueError
time.sleep(2)
status = mmwcas.mmw_arming_tda("outdoor0")
assert status==0,ValueError
time.sleep(2)
status = mmwcas.mmw_start_frame()
assert status==0,ValueError

time.sleep(record_duration)

status=mmwcas.mmw_stop_frame()
assert status==0,ValueError
status=mmwcas.mmw_dearming_tda()
assert status==0,ValueError
time.sleep(2)