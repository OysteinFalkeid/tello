from djitellopy import Tello

tello = Tello()
tello.connect()
tello.takeoff()
tello.move_forward()
tello.send_rc_control()