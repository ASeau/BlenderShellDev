import ifcopenshell
import ifcopenshell.util
# from ifcopenshell.util.selector import Selector

# imports ifc
from ifcopenshell.express.templates import get_inverse

ifc = ifcopenshell.open('TestModeltask.ifc')
# selector = Selector()
# Extract IfcTask
Task = ifc.by_type('IfcTask')
print(len(Task))

schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_task = schema.declaration_by_name('IfcTask')
ifc_task_att = ifc_task.all_attributes()
print(ifc_task)
print(ifc_task_att)

for i in range(len(Task)):
    TaskIDs = []
    TaskGuIDs = []
    TaskRef = []

    TaskEle = Task[i]
    TaskEleInfo = (TaskEle.get_info(1, 0))
    TaskEleGuID = TaskEleInfo.get("GlobalId")
    TaskEleID = TaskEleInfo.get("TaskId")

    TaskRef = TaskEleInfo.get("TaskId")
    TaskRef = TaskRef[0]
    #print(TaskRef)
    #TaskRef = get_inverse(TaskRef)

    TaskIDs.append(TaskEleID)
    TaskGuIDs.append(TaskEleGuID)
    #TaskRef.append(TaskEleRef)

    #print(TaskIDs)
    print(TaskGuIDs)
    print(TaskRef)
    #print(TaskEleInfo)