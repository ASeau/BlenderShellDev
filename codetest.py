import ifcopenshell
import bpy

ifc = ifcopenshell.open('TestModeltask.ifc')
print(ifc.schema)
# stepID
print(ifc.by_id(307))
# globalid
#print(ifc.by_guid('0EI0MSHbX9gg8Fxwar7lL8'))
# IFCType
RelAsProcess = ifc.by_type('IfcRelAssignsToProcess')
print(len(RelAsProcess))
print(RelAsProcess[0].is_a())
#

#IFC Attributes
schema = ifcopenshell.ifcopenshell_wrapper.schema_by_name("IFC2X3")
ifc_task = schema.declaration_by_name('IfcRelAssignsToProcess')
ifc_task_att = ifc_task.all_attributes()
print(ifc_task)
print(ifc_task_att)
