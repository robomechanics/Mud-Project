import can_control as cc
import time
import can

### Must have at the beginning of every controller ###
helper = cc.CanController()
listener = cc.CanListener(controller=helper, motor_type='AK80-8')
notifier = can.Notifier(helper.bus, [listener])
helper.power_on(motor_id=1)
helper.zero(motor_id=1)
time.sleep(2)
### --------------------------------------------------
try:
    for i in range(1, 12):
        helper.MIT_controller(motor_id=1, motor_type='AK80-8', position=i, velocity=0.5, 
                            Kp=4, Kd=1, I=0)
        time.sleep(1)

    for i in range(11, -12, -1):
        helper.MIT_controller(motor_id=1, motor_type='AK80-8', position=i, velocity=0.5, 
                            Kp=4, Kd=1, I=0)
        time.sleep(1)

    for i in range(-11, 1):
        helper.MIT_controller(motor_id=1, motor_type='AK80-8', position=i, velocity=0.5, 
                            Kp=4, Kd=1, I=0)
        time.sleep(1)

except KeyboardInterrupt as e:
    helper.power_off(motor_id=1)
    notifier.stop()
    helper.can_shutdown()

### Must have at the end of every controller ###
helper.power_off(motor_id=1)
notifier.stop()
helper.can_shutdown()
### --------------------------------------------