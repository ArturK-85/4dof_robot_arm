import math

def dxl_angle_to_degrees(dxl_angle):
    """ Change read servo dynamixel position code to degrees,
        and replace dynamixel angle notation from 0 to 300 degrees
        on to -150 to 150 degrees notation """

    angle_degrees = round(dxl_angle / 1023. * 300. - 150.0,1)
    return angle_degrees

def degrees_to_dxl_angle(angle_degrees):
    """ Change degrees to dynamixel position code, and replace
        from -150 to 150 notation to dynamixel 0 to 300 notation ,
        and convert on to dynamixel position code that you can
        send direct to servo """

    dxl_angle = math.floor((angle_degrees + 150.0) / 300. * 1023.)
    dxl_angle = abs(dxl_angle)
    return dxl_angle
