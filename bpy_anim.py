import bpy
import sys
import os
from math import *
from mathutils import *
from datetime import datetime,date,timedelta
import ifcopenshell
import ifcopenshell.util

#from blenderbim.bim.ifc import IfcStore
# from ifcopenshell.util.selector import Selector

#function: produces ifctaskGuID+ifcrelatedobjectGuID#

#imports ifc
path = os.path.abspath("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/CERB_Struc.ifc")
ifc = ifcopenshell.open(path)
#selector = Selector()
#Extract IfcRelAssignsToProcess
RelAssign = ifc.by_type('IfcRelAssignsToProcess')
#print(RelAssign,len(RelAssign))
schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_relassign = schema.declaration_by_name('IfcRelAssignsToProcess')
ifc_relassign_att = ifc_relassign.all_attributes()
#print(ifc_relassign)
#print(ifc_relassign_att)

#RelAssign

#Extract item ID

RelAssignProcess_dict = {}
RelAssign_Process = []
def getProcessItem():
    RelAssignProcess_dict = {}
    RelAssign_Process = []

    for i in range(len(RelAssign)):
        RelAssign_GuIDs = []
        RelAssign_item_GuIDs = []
        RelAssign_dict = []
        RelAssign_Process = []

        #RelAssign data
        RelAssignEle = RelAssign[i]
        RelInfo = (RelAssignEle.get_info(1,0))
        RelID = RelInfo.get("GlobalId")
        RelAssign_GuIDs.append(RelID)
        #print(RelAssign_GuIDs)

        #prints IFC task[2=task_name]
        RelProcess = RelInfo.get("RelatingProcess")
        RelProcessID = RelProcess[2]
        RelAssign_Process.append(RelProcessID)

        #prints task related items
        RelItems = RelInfo.get("RelatedObjects")
        # convert to ifcopenshell.entity_instance.entity_instance
        GuID_list = []

        for j in range(len(RelItems)):
            temp_item = RelItems[j]
            tag = temp_item[7]
            GuID_list.append(tag)

        #Create dict for hashing
        RelAssignpro_dict = dict(zip(RelAssign_Process,[GuID_list]))
        RelAssignProcess_dict.update(RelAssignpro_dict)

    return RelAssignProcess_dict

RelAssignProcess_dict = getProcessItem()

#####################################################################################
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

RelAssignTask_dict = {}

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
    RelAssignTask_dict.update(RelAssign_dict)
    #print("Resultant dictionary(ifctask+ifctimecontrol) is : " + str(RelAssign_dict))

#print("RelAssignTask_dict=",RelAssignTask_dict)

for i in range(len(Seq)):
    SeqEleIDs = []

ScheduleTime_dict = {}

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
    ScheduleTime_dict.update(Schedule_dict)

#####################################################################################

#print("RelAssignProcess_dict=", RelAssignProcess_dict)
#print("RelAssignTask_dict=", RelAssignTask_dict)
#print("ScheduleTime_dict=", ScheduleTime_dict)

#####################################################################################

#Combining dicts
d1 = RelAssignTask_dict
#print(RelAssignTask_dict)
d2 = ScheduleTime_dict
#print(ScheduleTime_dict)

TaskTime_dict = {}

for key in d1.keys():
    if key in d2.keys():
        #execute dict construc
        d3 = {d1[key]: d2[key]}
        TaskTime_dict.update(d3)

d3 = RelAssignProcess_dict
print(d3)
##RelAssign not reaching layer issue##

d4 = TaskTime_dict
print(d4)

Anim_dict = {}

for key in d3.keys():
    if key in d4.keys():
        #execute dict construc
        timedata = ""
        timedata = d4[key]
        timedata = timedata.split(",")
        d5 = [d3[key], timedata]
        d6 = {key: d5}
        Anim_dict.update(d6)

print("Anim_dict =", Anim_dict)
##Progress: Anim is able to get all tasks##

######################################################################################
#construct dict
#dict = {taskname: [[relatedobjtag], [start time, end time, duration]],taskname: [[relatedobjtag], [start time, end time, duration]]}
#get obj via tagID
'''
for taskname in Anim_dict.keys():
    item_GUIDs = Anim_dict[taskname][0]
    for GUIDs in item_GUIDs:
        bpy.context.scene.BIMSearchProperties.global_id = "GUIDs"
        print()

##function(GUID)

##add to collection

def CreateCollections():
    collection_name = []
    for key in Anim_dict.keys():
        collection_name.append(key)

    #print(search_list)
    list_length = [len(x) for x in collection_name]
    print(collection_name)
'''
"""
name_list =[]

for obj in bpy.context.scene.objects:
    for i in range(len(search_list)):
        if any(x in obj.name for x in search_list[i]):
            name_list.append(obj.name)

print(name_list)

tagxitem_dict = dict(zip(search_list, name_list))

###resultingdict = tagID:item_name

#replace item name into Anim_dict
for key in Anim_dict.keys():
    for key2 in tagxitem_dict.keys():
        tag = [Anim_dict[key]][0][0]
        if tag == key2:
            name = tagxitem_dict[key2]
            [Anim_dict[key]][0][0] = name

#print("Anim_dict =", Anim_dict)
#Anim_dict = {task:[objname[start,end,dur]}
###IFCDATAASSOCIATIONEND###
"""

###BlenderAnimation###
#Animation initialization
# set first and last frame index
'''
total_time = 2*pi # Animation should be 2*pi seconds long
fps = 24 # Frames per second (fps)
bpy.context.scene.frame_start = 0
bpy.context.scene.frame_end = int(total_time*fps)+1
'''
#Select obj for inital key frame
rel_obj = []
for key in Anim_dict.keys():
    n = [Anim_dict[key]][0][0]
    rel_obj.append(n)

print(rel_obj)

temp_dict = {}
name_dict = {}
for obj in bpy.data.objects:
    #print(obj.name_full)
    if (obj.type == "MESH"):
        for i in range(len(rel_obj)):
            for j in range(len(rel_obj[i])):
                if rel_obj[i][j] in obj.name_full:
                    print("Matched",rel_obj[i][j],obj.name_full)
                    temp_dict = dict(zip([rel_obj[i][j]],[obj.name_full]))
                    name_dict.update(temp_dict)
print(name_dict)
    #if (obj.BIMAttrubuteProperties.attributes is not None):
    #    tag = obj.BIMAttrubuteProperties.attributes[7].string_value
    #    objtag_dict = dict(zip(obj,tag))
    #else:
    #    print("notBIMItems")

#print(objtag_dict)

#for i in range(len(rel_obj)):
#tag = obj.BIMAttrubuteProperties.attributes[7].string_value
#if (tag == rel_obj[i]):
#print(tag,rel_obj[i])

#temp_dict = dict(zip(rel_obj[i], [obj.name]))
#name_dict.update(temp_dict)
#print(name_dict,len(name_dict))
#if any(x in obj.name for x in str(rel_obj)):
#bpy.data.objects.BIMAttrubuteProperties.attributes[7].string_value
for ob in bpy.context.scene.objects:
    ob.animation_data_clear()

for o in name_dict.values():
    obj = bpy.context.scene.objects.get(o)
    obj.select_set(True)

for obj in bpy.context.selected_objects:
    obj.hide_viewport = True
    obj.hide_render = True
    obj.keyframe_insert(data_path="hide_viewport",frame=0)
    obj.keyframe_insert(data_path="hide_render", frame=0)
    print(obj,'objects hidden')

#initialization complete
#Animate according to task steps
current_frame = 0
#animation logic
#determine max_duration = lastkeyend - firstkeystart
#then from calender determine actual duration from date
#index the date to dict
#for ket in anime_dict, if keystart=index,then index = frame number

date_list = []
date_dict = {}
for key in Anim_dict.keys():
    start_date = [Anim_dict[key]][0][1][0]
    end_date = [Anim_dict[key]][0][1][1]
    date_list.append(start_date)
    date_list.append(end_date)

print(date_list)
sdate = date.fromisoformat(min(date_list))  # start date
edate = date.fromisoformat(max(date_list))   # end date
print(sdate,edate)

delta = edate - sdate       # as timedelta
print(delta)

for i in range(delta.days + 1):
    day = sdate + timedelta(days=i)
    date_dict.update({str(day):i+5})

print(date_dict)

for key in Anim_dict.keys():
    print('key=',key)
    start_date = [Anim_dict[key]][0][1][0]
    #print(start_date)
    end_date = [Anim_dict[key]][0][1][1]
    #print(end_date)
    duration = [Anim_dict[key]][0][1][2]
    duration = int(float(duration))

    current_frame = date_dict[start_date]*5
    #current_frame += date_dict[start_date]
    #magnify/strechtime
    print('frame assigned',current_frame)

    obj_tag = [Anim_dict[key]][0][0]
    #print(obj_tag)

    for tag in obj_tag:
        ob = name_dict[tag]
        obj = bpy.context.scene.objects.get(ob)
        print(obj)
        obj.hide_viewport = False
        obj.keyframe_insert(data_path="hide_viewport", frame=current_frame)
        print(key,obj.name,'activated')

    print("taskended")
    #print(current_frame)

total_time = 2 * pi  # Animation should be 2*pi seconds long
fps = 24  # Frames per second (fps)
bpy.context.scene.frame_start = 0
bpy.context.scene.frame_end = int(current_frame) + 100
#bpy.context.scene.frame_end = int(total_time * fps) + 1
'''
    for ob in :
        obj = bpy.context.scene.objects.get(ob)
        print(obj.name)
        obj.hide_viewport = False
        obj.keyframe_insert(data_path="hide_viewport", frame=current_frame)

print(current_frame)
temp_namelist = []
    for tag in obj_tag:
        for name in namelist:
            if any(x in name for x in tag):
                temp_namelist.append(name)
'''
'''
    temp_namelist = []
'''
'''

    for obj in bpy.context.scene.objects:
        for i in range(len(obj_tag)):
            if any(x in obj.name for x in obj_tag[i]):
                temp_namelist.append(obj.name)

    temp_namelist = list(dict.fromkeys(temp_namelist))
    print(len(temp_namelist))
    print(key)
'''
