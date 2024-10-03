from math import atan2, asin, sqrt, acos
from geometry_msgs.msg import Quaternion

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):
        with open(self.filename, 'a') as file:
            vals_str=""

            for value in values_list:
                vals_str += f"{value},"
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table


def euler_yaw_from_quaternion(quat: Quaternion):
    """
    Convert quaternion (w in last place) to euler yaw angle [rad]
    quat = [x, y, z, w]
    """
    # Just derived this from the quaternion to rotation matrix equation in modeling_iv lecture
    # r11 = quat.w**2 + quat.x**2 - quat.y**2 - quat.z**2
    # r21 = 2.0 * (quat.w * quat.z + quat.x * quat.y)    
    # yaw = atan2(r21, r11)

    # NOTE: The quaternion seems to have unique pairs of z and w
    # (signs flipped) when rotating to the same orientation
    # e.g. a full spin will return an angle of -2*PI -> +2*PI
    # instead of returning back to 0

    # Commented code above has it jump from -PI to +PI halfway through a full rotation

    # Code below has it go down all the way to -2PI during the first rotation,
    # jump up to 2PI at the start of the second rotation, and come back down to 0 rad at the end of two full rotations
    # Not sure which one is preferred or more "correct"
    yaw = 2 * atan2(quat.z, quat.w)

    return yaw
