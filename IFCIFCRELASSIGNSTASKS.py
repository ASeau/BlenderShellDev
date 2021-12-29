import os
from datetime import date
import ifcopenshell
import ifcopenshell.util
# from ifcopenshell.util.selector import Selector
from ifcopenshell.express.templates import get_inverse

#function: produces ifctaskGuID+ifcrelatedobjectGuID#

#imports ifc
path = os.path.abspath("D:/User Data/Documents/NTU109-2/BlenderShellDev/TestModeltask.ifc")
ifc = ifcopenshell.open(path)
#selector = Selector()
#Extract IfcRelAssignsToProcess

schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_relassign = schema.declaration_by_name('IFCRELASSIGNSTASKS')
ifc_relassign_att = ifc_relassign.all_attributes()
print(ifc_relassign)
print(ifc_relassign_att)

RelAssign = ifc.by_type('IFCRELASSIGNSTASKS')
print(len(RelAssign))

RelAssignTask_dict = []

for i in range(len(RelAssign)):
    RelAssignIDs = []
    RelAssignTasks = []
    RelAssignTime = []

    RelAssignEle = RelAssign[i]
    RelAssignEleInfo = (RelAssignEle.get_info(1, 0))
    #print("RelAssignEleInfo=",RelAssignEleInfo)

    RelAssignEleID = RelAssignEleInfo.get("GlobalId")
    RelAssignIDs.append(RelAssignEleID)

    #IFCTASK name
    RelAssignTask = RelAssignEleInfo.get("RelatedObjects")
    RelAssignTaskEle = RelAssignTask[0]
    RelAssignTaskEleName = RelAssignTaskEle[2]
    RelAssignTasks.append(RelAssignTaskEleName)
    #print(RelAssignTasks)

    #IFCTIMECONTROL
    RelAssignTimeEle = RelAssignEleInfo.get("TimeForTask")
    RelAssignTimeEleID = RelAssignTimeEle.GlobalId
    #print(RelAssignTimeEleID)
    #print(RelAssignTimeEleID)
    RelAssignTime.append(RelAssignTimeEleID)
    #print(RelAssignTask)
    # Create dict for hashing
    RelAssign_dict = dict(zip(RelAssignTasks, RelAssignTime))
    RelAssignTask_dict.append(RelAssign_dict)
    #print("Resultant dictionary(ifctask+ifctimecontrol) is : " + str(RelAssign_dict))
print("RelAssignTask_dict=",RelAssignTask_dict)