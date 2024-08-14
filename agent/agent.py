import math
import random
import xml.etree.ElementTree as ET
import numpy as np
from croblink import CRobLinkAngs
import sys


CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, path_file):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.path_file = path_file
        self.halfwall_size = 0.1
        self.robot_radius = 0.5
        self.pos = None
        self.rob_coords = []
        self.last_coord = None
        self.next_coord = None        
        self.Vl = 0
        self.Vr = 0
        self.out_Vl = 0
        self.out_Vr = 0 
        self.angle_radians = 0
        self.prev_ori = None
        self.prev_action = None
        self.vector = 0
        self.angle = 0
        self.target_ori = 0

    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
     
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()        
                    
        state = 'stop'
        stopped_state = 'return'
        
        # Load initial position
        init_pos,list_coords = self.localization()    
        
        # Add initial position to rob
        if self.pos == None:
            self.pos = init_pos
        
        # Add initial position
        if init_pos not in self.rob_coords:
            self.rob_coords.append(init_pos)
        
        while True:
            if state != 'return':
                self.prev_ground = 0
            else:
                self.prev_ground = self.measures.ground    
            
            #read sensors
            self.readSensors()   
            
            # call following line method
            if state == 'stop' and self.measures.start:
                state = stopped_state
            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)

                # Navigate trough the path of txt
                action = self.control(list_coords)  
                
                # Perform action       
                self.navigate(action)
                            
                # Estimate position with mov.model
                self.mov_model()
                                
                # Get explored coordinates
                self.retrieve_coordinates()  
                              
    def localization(self):
        """ Load path.txt and create list of coordinates """
        list_coordinates = []
        init_pos = (0,0)
        list_coordinates.append(init_pos)
        with open(self.path_file) as f:
            for i, line in enumerate(f):
                coordinates = list(map(int, line.strip().split()))
                list_coordinates.append(tuple(coordinates))
        return init_pos,list_coordinates

    def calculate_ang(self, x1, y1, x2, y2):
        """ Calculate the angle (in degrees) between two positions (x1, y1) and (x2, y2) """
        angle_rad = math.atan2(y2 - y1, x2 - x1)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_ang_diff(self, ang1, ang2):
        """ Calculate the difference between two angles (in degrees) """
        angle_diff = ang2 - ang1
        # Angle within the range of -180 to 180 degrees
        while angle_diff < -180:
            angle_diff += 360
        while angle_diff > 180:
            angle_diff -= 360
        return angle_diff
               
    def control(self, list_coords):
        """ Choose action according to next coord """
        # No coordinates retrieved
        if not self.rob_coords:
            return "no coords available"
        
        # Calculate the difference between rob_coords length and list_coords length
        rob_coords_size = len(self.rob_coords)
        list_coords_size = len(list_coords)
        size_diff = rob_coords_size - list_coords_size

        # Use the difference to determine the next index in list_coords
        next_idx = size_diff % list_coords_size
          
        # Find the next coordinate based on the last position in rob_coords
        self.last_coord = self.rob_coords[-1]

        # Run trough list of coordinates
        if next_idx < len(list_coords) and next_idx > 0:
            # self.idxs.append(next_idx)
            self.next_coord = list_coords[next_idx]
            #print("Next id from path: ",next_idx)
            print("Curr Coordinate: ", self.last_coord)
            print("Next Coordinate: ", self.next_coord)
            
            current_x, current_y = self.last_coord
            next_x, next_y = self.next_coord

            # Calculate angle between current coord and next coord
            angle_to_next_pos = self.calculate_ang(current_x, current_y, next_x, next_y)

            # Calculate the diff between the angle to the next pos and the compass readings
            angle_diff = self.calculate_ang_diff(angle_to_next_pos, self.measures.compass)
            
            # Determine the action based on the angle difference
            if abs(angle_diff) > 10:
                if angle_diff > 0:
                    return "rotate right"
                else:
                    return "rotate left"
            else:
                return "move"        
        else:
            return "finished" 
            
    def navigate(self,action):
        """ Navigate according to path from txt"""   

        # Check distance from sensors
        if self.measures.irSensor[0] != 0:
            front_dist = (1/self.measures.irSensor[0])-self.halfwall_size#+self.robot_radius
        else:
            front_dist = 0
        if self.measures.irSensor[1] != 0:    
            left_dist = (1/self.measures.irSensor[1])-self.halfwall_size#+self.robot_radius
        else:
            left_dist = 0
        if self.measures.irSensor[2] != 0:
            right_dist = (1/self.measures.irSensor[2])-self.halfwall_size#+self.robot_radius
        else:
            right_dist = 0
        if self.measures.irSensor[3] != 0:
            rear_dist = (1/self.measures.irSensor[3])-self.halfwall_size#+self.robot_radius
        else:
            rear_dist = 0
        
        # Based on distances to wall, perform actions
        if action == "rotate right":
            self.rotate(1)
        elif action == "rotate left":
            self.rotate(-1)
        elif action == "move":
            if left_dist < 0.15:
                print('Action: Rotate slowly right')
                self.driveMotors(0.05,0.03)
                self.Vl = 0.05
                self.Vr = 0.03
            elif right_dist < 0.15:
                print('Action: Rotate slowly left')
                self.Vl = 0.03
                self.Vr = 0.05
                self.driveMotors(0.03,0.05)
            elif left_dist == 0 or right_dist == 0 or front_dist == 0 or rear_dist == 0:
                print('Action: Rotate slowly right')
                self.driveMotors(0.05,-0.05)
                self.Vl = 0.05
                self.Vr = -0.05
            else:
                print('Action: Move')
                self.Vl = 0.05
                self.Vr = 0.05
                self.driveMotors(0.05,0.05)
        
        elif action == "finished":
            print("No more coordinates in path.")
            self.Vl = 0
            self.Vr = 0
            self.driveMotors(0,0)
            exit()          
        else:
            print('Lost')
            self.Vl = -0.5
            self.Vr = 0.5
            self.driveMotors(-0.5,0.5)

    def rotate(self, dir):
        
        if self.prev_ori == None:
            self.prev_ori = round(self.measures.compass)
        # Calculate the vector from the current coordinate to the next coordinate
        if self.prev_action == None:
            self.vector = (self.next_coord[0] - self.last_coord[0], self.next_coord[1] - self.last_coord[1])
            # Calculate the angle between the current vector and the y-axis
            self.angle = math.degrees(math.atan2(self.vector[1], self.vector[0]))
            # Calculate the target orientation
            self.target_ori = self.angle-90*dir
            # Adjust the target orientation to be within the range
            self.target_ori = (self.target_ori + 360) % 360
            # Action flag
            self.prev_action = True
        
        # Rotate until the target orientation is reached
        if self.measures.compass < self.target_ori:
            print("Action: Rotating")
            self.driveMotors(0.05*dir, -0.05*dir)
            self.Vl = 0.05*dir
            self.Vr = -0.05*dir     
        else:
            # Update the previous orientation
            print("Previous orientation updated")
            self.prev_ori = round(self.measures.compass)
            self.prev_action = None    

    def mov_model(self):
        """ Movement model with IIR filter """
        
        # Read compass and convert angle to radians
        self.angle_radians = math.radians(self.measures.compass)   
        
        # Limit max speed
        max_speed = 0.05
        
        # Add noise
        motors_noise = random.gauss(1,max_speed**2)
        
        # Load previous position
        x_t0, y_t0 = self.pos
        
        # Calculate power component
        self.out_Vl_t = ((self.Vl + self.out_Vl) / 2.0) * motors_noise
        self.out_Vr_t = ((self.Vr + self.out_Vr) / 2.0) * motors_noise 
        
        # Limit the wheel speeds to the maximum speed
        self.out_Vl_t = min(self.out_Vl_t, max_speed)
        self.out_Vr_t = min(self.out_Vr_t, max_speed)
        
        # Linear vel
        lin = (self.out_Vr_t + self.out_Vl_t) / 2.0
        #rot = self.out_Vr_t - self.out_Vl_t
        
        # Calculate linear component:
        x_t1 = x_t0 + lin * np.cos(self.angle_radians)
        y_t1 = y_t0 + lin * np.sin(self.angle_radians)

        # Calculate rotation component:
        #self.out_teta_t = self.out_teta + rot
        
        # Return the estimated next pose
        self.pos = (x_t1, y_t1)   #self.angle_radians if needed
        
        # Update position
        self.out_Vl = self.out_Vl_t
        self.out_Vr = self.out_Vr_t
        #self.out_teta = self.out_teta_t
        
    def retrieve_coordinates(self):
        """ Retrieve coordinates based on estimated positions """
        
        # Initialize readings
        distance_x = 0
        distance_y = 0
        x = round(self.pos[0],1)
        y = round(self.pos[1],1)
    
        # Get distance absolute value 
        distance_x = abs(x - self.rob_coords[-1][0])
        distance_y = abs(y - self.rob_coords[-1][1])        
        # Measure distance compared to last coord
        if distance_x > 1.90 or distance_y > 1.90:
            x = round(x)
            y = round(y)
            # Add coords to list
            self.rob_coords.append((x, y))

        
class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1

ir_pos = np.array([[0.5,0], [0,0.5],[0,-0.5],[-0.5,0]])
ir_ang = np.array([0, math.pi/2, -math.pi/2, math.pi])

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
path_file = 'path.txt'

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--xy" or sys.argv[i] == "-x") and i != len(sys.argv) - 1:
        path_file = sys.argv[i + 1]
    elif (sys.argv[i] == "--cell" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        ini_line,ini_col = sys.argv[i + 1].split(sep='x')
        print('starting at line',ini_line,'column',ini_col)
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':

    rob=MyRob(rob_name, pos, ir_ang*180/np.pi, host, path_file)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
