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
    r11 = quat.w**2 + quat.x**2 - quat.y**2 - quat.z**2
    r21 = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    
    yaw = atan2(r21, r11)

    return yaw
