import os
from datetime import date
import ifcopenshell
import ifcopenshell.util
# from ifcopenshell.util.selector import Selector

# imports ifc
path = os.path.abspath("D:/User Data/Documents/NTU109-2/BlenderShellDev/TestModeltask.ifc")
ifc = ifcopenshell.open(path)
# selector = Selector()
schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_schedule = schema.declaration_by_name('IfcSCHEDULETIMECONTROL')
ifc_schedule_att = ifc_schedule.all_attributes()
#print(ifc_schedule)
#print(ifc_schedule_att)

ifc_seq = schema.declaration_by_name('IFCRELSEQUENCE')
ifc_seq_att = ifc_seq.all_attributes()
#print(ifc_seq)
#print(ifc_seq_att)
# Extract IfcTask
Schedule = ifc.by_type('IfcSCHEDULETIMECONTROL')
Seq = ifc.by_type('IFCRELSEQUENCE')
#print(len(Schedule))
#print(len(Seq))

schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_relassign = schema.declaration_by_name('IFCRELASSIGNSTASKS')
ifc_relassign_att = ifc_relassign.all_attributes()
#print(ifc_relassign)
#print(ifc_relassign_att)

RelAssign = ifc.by_type('IFCRELASSIGNSTASKS')
#print(len(RelAssign))

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
    RelAssign_dict = dict(zip(RelAssignTime, RelAssignTasks))
    RelAssignTask_dict.append(RelAssign_dict)
    #print("Resultant dictionary(ifctask+ifctimecontrol) is : " + str(RelAssign_dict))

#print("RelAssignTask_dict=",RelAssignTask_dict)

for i in range(len(Seq)):
    SeqEleIDs = []

ScheduleTime_dict = []

for i in range(len(Schedule)):
    ScheduleEleIDs = []
    #EarlyStartDate = []
    #LateStartDate = []
    ScheduleStartDate = []

    #ActualFinishDate = []
    #EarlyFinish = []
    #LateFinish= []
    ScheduleFinish= []
    ScheduleDuration= []

    ScheduleEle = Schedule[i]
    ScheduleEleInfo = (ScheduleEle.get_info(1, 0))
    ScheduleEleID = ScheduleEleInfo.get("GlobalId")
    ScheduleEleIDs.append(ScheduleEleID)
    #print("GuIDs=",ScheduleEleIDs)

    ScheduleEleSS = ScheduleEleInfo.get("ScheduleStart")
    SSdate = ifc.traverse(ScheduleEleSS,-1)
    SSdate = SSdate[1]
    SSdate = date(SSdate.YearComponent, SSdate.MonthComponent, SSdate.DayComponent)
    #print("SS=",SSdate)

    ScheduleEleSF = ScheduleEleInfo.get("ScheduleFinish")
    SFdate = ifc.traverse(ScheduleEleSF, -1)
    SFdate = SFdate[1]
    SFdate = date(SFdate.YearComponent, SFdate.MonthComponent, SFdate.DayComponent)
    #print("SF=", SFdate)

    ScheduleEleSD = ScheduleEleInfo.get("ScheduleDuration")
    #SD = ifc.traverse(ScheduleEleSD, -1)
    SD = ScheduleEleSD
    SWorkDay= SD/60/60/8
    #print("SWorkDay=",SWorkDay)

    SS_SF_SWorkDay = str(SSdate) + ',' + str(SFdate)+ ',' + str(SWorkDay)
    SS_SF_SWorkDay = SS_SF_SWorkDay.split()
    #print(SS_SF_SWorkDay)

    Schedule_dict = dict(zip(ScheduleEleIDs, SS_SF_SWorkDay))
    ScheduleTime_dict.append(Schedule_dict)
    #print("Resultant dictionary is : " + str(Schedule_dict))

#print("RelAssignTask_dict=",RelAssignTask_dict)
#print("ScheduleTime_dict=",ScheduleTime_dict)
#Compare&CombineDict



    #ScheduleEleclass = ScheduleEle.is_a()
    #print(ScheduleEleclass)
    #type = type(ScheduleEle)
    #print("t=",type)
    #print(ScheduleEle)
    #TaskIDs.append(TaskEleID)
    #TaskGuIDs.append(TaskEleGuID)

    #print(TaskIDs)
    #print(TaskGuIDs)

#print(ScheduleEleInfo)
