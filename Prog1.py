from robolink import *    # API to communicate with RoboDK for simulation and offline/online programming
from robodk import *      # Robotics toolbox for industrial robots
from robodk.robomath import *       # import the robotics toolbox
import time

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()
robot = RDK.Item('Robot', ITEM_TYPE_ROBOT)
tool = RDK.Item('Tool 1', ITEM_TYPE_TOOL)
reff = RDK.Item('WorkSpace', ITEM_TYPE_TARGET)
home = RDK.Item('Home', ITEM_TYPE_TARGET)
partref = RDK.Item('brick')

#----------------------------------------------------------------------------------------------------------

class Brick:
    def __init__(self, color, amt, RGB):
        self.col = color
        self.amt = amt
        self.RGB = RGB

#----------------------------------------------------------------------------------------------------------
import csv
names = ["brick1.csv","brick2.csv","brick3.csv","brick4.csv","brick5.csv"]
newList = []

for i in names:
    path_file = getOpenFile("brick1.csv/")
    if not path_file:
        print("Nothing selected")
        quit()
    program_name = getFileName(path_file)
    data = LoadList(path_file)
    col1 = data

    col1.pop(0)
    for list in col1:
        list.pop(0)
    for i in col1:
        for x in range(len(i)):
            i[x] = int(i[x])
            i[x] = i[x]/3
    newList.append(col1)
print(newList)


#----------------------------------------------------------------------------------------------------------

BrickWide = 32
BrickHigh = 19.2
Space = BrickWide/2
ApproachHigh = 70

Colors = [Brick('Red',2,[1, 0, 0, 1]), Brick('Green',1,[0, 1, 0, 1]), Brick('Blue',4,[0, 0, 1, 1]),Brick('White',1,[1, 1, 1, 1]),Brick('Yellow',6,[1, 1, 0, 1])]
Cols = [[1, 0, 0, 1],[0, 1, 0, 1],[0, 0, 1, 1],[1, 1, 1, 1],[1, 1, 0, 1]]
Brick = []

cont = 0

for x in range(len(Colors)):
    Brick.append([])
    for i in range(Colors[x].amt):
        Brick[x].append([newList[x][i][0],newList[x][i][1],0,Cols[x],(newList[x][i][3]/90)*3.14])



Fig1 = [Brick[2][0],Brick[4][0]]
Fig2 = [Brick[2][1],Brick[0][0],Brick[4][1]]
Fig3 = [Brick[4][2],Brick[0][1],Brick[4][3]]
Fig4 = [Brick[1][0],Brick[4][4],Brick[2][2]]
Fig5 = [Brick[2][3],Brick[3][0],Brick[4][5]]

Figs = [Fig1, Fig2, Fig3, Fig4, Fig5]

#Define end location
EndLocation = []
for i in range(len(Figs)):
    EndLocation.append(reff.Pose()*transl(0,(BrickWide+Space)*i,0))

#Reset position of robot
robot.setJoints([45,-90,90,-90,-90,0])
#Open tool
robot.setDO(0, 1)
time.sleep(0.1)
robot.setDO(0, 0)


RDK.Render(False)
#Remove old bricks
all_objects = RDK.ItemList(ITEM_TYPE_OBJECT, False)
for item in all_objects:
        if item.Name().startswith('Part '):
            item.Delete()

#----------------------------------------------------------------------------------------------------------

#Place bricks
for x in range(len(Figs)):
    for i in range(len(Figs[x])):
        partref.Copy()
        newpart = reff.Paste()
        newpart.Scale([BrickWide/100, BrickWide/100, BrickHigh/100])
        newpart.setName('Part ' + str(i+1))
        newpart.setVisible(True, False)
        newpart.setPose(transl((Figs[x][i][0],Figs[x][i][1],Figs[x][i][2]))*rotz(Figs[x][i][4]))
        print(Figs[x][i][4])
        newpart.Recolor(Figs[x][i][3])
RDK.Render(True)

#Define pos of bricks
FigLocs = []
for x in range(len(Figs)):
    Loc = []
    for i in range(len(Figs[x])):
        location = reff.Pose()*transl(Figs[x][i][0],Figs[x][i][1],Figs[x][i][2])*rotz(Figs[x][i][4])
        Loc.append(location)
    FigLocs.append(Loc)

for x in range(len(FigLocs)):
    for i in range(len(FigLocs[x])):
        #Pick up
        approach = FigLocs[x][i]*transl(0,0,-ApproachHigh)
        robot.MoveL(approach)
        robot.MoveL(FigLocs[x][i])
        tool.AttachClosest()
        robot.setDO(1, 1)
        time.sleep(0.1)
        robot.setDO(1, 0)
        robot.MoveJ(approach)

        #Place
        approach = EndLocation[x]*transl(0,0,-ApproachHigh+(-BrickHigh*i))
        robot.MoveL(approach)
        robot.MoveL(EndLocation[x]*transl(0,0,-BrickHigh*i))
        tool.DetachAll()
        robot.setDO(0, 1)
        time.sleep(0.1)
        robot.setDO(0, 0)
        robot.MoveL(approach)

#Reset position of robot
robot.MoveJ(home)
