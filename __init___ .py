# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

bl_info = {
    "name": "LiDAR-Tools",
    "author": "Brian Hynds",
    "version": (1, 0, 1),
    "blender": (2, 93, 0),
    "location": "File > Import > LASdata",
    "description": "LiDAR Importer",
    "warning": "",
    "doc_url": "",
    "category": "Import-Export",
}

import bpy
import laspy
import numpy as np
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, EnumProperty
from bpy.types import Operator
from mathutils import Vector

def get_attribute(header, attr_name, default="N/A"):
    if hasattr(header, attr_name):
        return str(getattr(header, attr_name))
    else:
        return default


class IMPORT_OT_las_data(Operator, ImportHelper):
    bl_idname = "import_scene.las_data"
    bl_label = "Import LAS/LAZ data"
    bl_options = {'PRESET', 'UNDO'}

    filename_ext = ".las;.laz"  # Add support for LAZ files
    filter_glob: StringProperty(default="*.las;*.laz", options={'HIDDEN'})  # Update the filter_glob to include LAZ files



    def execute(self, context):
        with laspy.open(self.filepath) as infile:
            las = infile.read()
            points = np.vstack((las.x, las.y, las.z)).T
            all_attr_name = list(las.point_format.dimension_names)
            input_attributes = []
            list_attr_name = []

            # Normalize selected attribute
            for attr in all_attr_name:
                if attr in ['X', 'Y', 'Z'] or np.sum(list(las[attr])) == 0:
                    continue
                list_attr_name.append(attr)
                input_attributes.append(las[attr])

            mins = np.asarray(input_attributes).min(axis=1)
            maxs = np.asarray(input_attributes).max(axis=1)

            # Prepare metadata
            lidar_info = {
                'filepath': self.filepath,
                'point_count': infile.header.point_count,
                'mins': mins,
                'maxs': maxs
            }

        self.import_points_as_mesh(context, points, lidar_info, input_attributes, list_attr_name)

        return {'FINISHED'}

    @staticmethod
    def add_geometry_nodes_points_modifier(obj, material):
        # ——— 4) Build the Geometry Nodes tree ———
        group = bpy.data.node_groups.new("PointCloudGN", 'GeometryNodeTree')

        # add I/O sockets (Blender 4.0+ API now needs description=)
        group.interface.new_socket(
            name="Geometry",
            description="Input geometry",
            in_out='INPUT',
            socket_type='NodeSocketGeometry'
        )
        group.interface.new_socket(
            name="Geometry",
            description="Output geometry",
            in_out='OUTPUT',
            socket_type='NodeSocketGeometry'
        )

        nodes = group.nodes
        links = group.links

        # create nodes
        ng_in  = nodes.new('NodeGroupInput');           ng_in.location  = (-300,  0)
        m2p    = nodes.new('GeometryNodeMeshToPoints'); m2p.location   = ( -50,  0)
        setm   = nodes.new('GeometryNodeSetMaterial');  setm.location  = ( 200,  0)
        ng_out = nodes.new('NodeGroupOutput');          ng_out.location = ( 450,  0)

        # configure and link
        m2p.inputs['Radius'].default_value = 0.05  # adjust point size
        links.new(ng_in.outputs['Geometry'], m2p.inputs['Mesh'])
        setm.inputs['Material'].default_value = material
        links.new(m2p.outputs['Points'],    setm.inputs['Geometry'])
        links.new(setm.outputs['Geometry'], ng_out.inputs['Geometry'])

        # attach the GN modifier
        mod = obj.modifiers.new("PointCloudGN", 'NODES')
        mod.node_group = group


    @staticmethod
    def assign_vertex_color_material(obj, vcol_layer_name):
        # Check if attribute exists
        if vcol_layer_name not in [attr.name for attr in obj.data.attributes]:
            # Fallback to first available attribute
            # attr_names = obj.get("lidar_attr_names", "").split(",")
            attr_names = obj.get("lidar_attr_names", "")
            if attr_names and attr_names[0]:
                vcol_layer_name = attr_names[0]
            else:
                return  # No valid attribute, do nothing

        # Remove all existing materials
        obj.data.materials.clear()

        # Create new material
        if "LAS_Material" not in bpy.data.materials:
            mat = bpy.data.materials.new(name="LAS_Material")
            mat.use_nodes = True
            nodes = mat.node_tree.nodes
            links = mat.node_tree.links

            # Clear default nodes
            for node in nodes:
                nodes.remove(node)

            # Add necessary nodes
            output = nodes.new(type='ShaderNodeOutputMaterial')
            diffuse = nodes.new(type='ShaderNodeBsdfDiffuse')
            color_ramp = nodes.new(type='ShaderNodeValToRGB')
            map_range = nodes.new(type='ShaderNodeMapRange')
            attr_node = nodes.new(type='ShaderNodeAttribute')

            attr_node.attribute_name = vcol_layer_name  # Use the vertex color layer name
            # Connect nodes
            links.new(attr_node.outputs['Color'], map_range.inputs['Value'])
            links.new(map_range.outputs['Result'], color_ramp.inputs['Fac'])
            links.new(color_ramp.outputs['Color'], diffuse.inputs['Color'])
            links.new(diffuse.outputs['BSDF'], output.inputs['Surface'])

            # Position nodes nicely
            attr_node.location = (-300, 0)
            map_range.location = (-150, 0)
            color_ramp.location = (0, 0)
            diffuse.location = (150, 0)
            output.location = (300, 0)

        else:
            mat = bpy.data.materials['LAS_Material']
            # Find the first Attribute node, or create one if missing
            attr_node = None
            for node in mat.node_tree.nodes:
                if node.type == 'ATTRIBUTE':
                    attr_node = node
                    break
            if attr_node is None:
                attr_node = mat.node_tree.nodes.new(type='ShaderNodeAttribute')
            attr_node.attribute_name = vcol_layer_name  # Use the vertex color layer name

            # Find the first Map Range node, or create one if missing
            map_range = None
            for node in mat.node_tree.nodes:
                if node.type == 'MAP_RANGE':
                    map_range = node
                    break
            if map_range is None:
                map_range = mat.node_tree.nodes.new(type='ShaderNodeMapRange')

            idx = obj.get("lidar_attr_names", "").split(",").index(vcol_layer_name)
            map_range.inputs['From Min'].default_value = obj['mins']
            map_range.inputs['From Max'].default_value = obj['maxs']

        # Assign material to the object
        obj.data.materials.append(mat)

        # Only add the GN modifier if it doesn't exist yet
        if not any(m.type == 'NODES' and m.name == "PointCloudGN" for m in obj.modifiers):
            IMPORT_OT_las_data.add_geometry_nodes_points_modifier(obj, mat)

        
    def import_points_as_mesh(self, context, points, lidar_info, input_attributes, list_attr_name):
        # Ensure points are Python float tuples
        points = np.asarray(points, dtype=np.float32)
        points_list = [tuple(pt) for pt in points]

        # Create a new mesh object
        mesh = bpy.data.meshes.new("LAS Data")
        obj = bpy.data.objects.new("LAS Data", mesh)
        # Link the mesh to the scene
        context.collection.objects.link(obj)
        context.view_layer.objects.active = obj
        obj.select_set(True)
    
        # Store LiDAR info in the object as separate custom properties
        obj['lidar_filepath'] = lidar_info['filepath'] 
        obj['lidar_point_count'] = lidar_info['point_count']
        # obj['mins'] = get_attribute(lidar_info['mins'], 'mins', lidar_info['mins'])
        # obj['maxs'] = get_attribute(lidar_info['maxs'], 'maxs', lidar_info['maxs'])

        # Create mesh vertices from points
        mesh.from_pydata(points_list, [], [])
        mesh.update()  # Update mesh data

        # Assign vertex colors
        if list_attr_name:
            for attr_array, attr_name in zip(input_attributes, list_attr_name):
                arr = np.asarray(attr_array, dtype=np.float32)

                mesh.attributes.new(name=attr_name, type='FLOAT', domain='POINT')
                mesh.attributes[attr_name].data.foreach_set("value", arr)
            
            obj["lidar_attr_names"] = ",".join(list_attr_name)
        
        # Assign material using either RGB or first attribute
        selected_attr = context.scene.lidar_selected_attr if context.scene.lidar_selected_attr else list_attr_name[0]
        idx = obj.get("lidar_attr_names", "").split(",").index(selected_attr)
        obj['mins'] = get_attribute(lidar_info['mins'], 'mins', lidar_info['mins'])[idx]
        obj['maxs'] = get_attribute(lidar_info['maxs'], 'maxs', lidar_info['maxs'])[idx]

        IMPORT_OT_las_data.assign_vertex_color_material(obj, selected_attr)
        
        obj.location = Vector((0, 0, 0))

        mesh.update()


    def get_lidar_attr_items(self, context):
        obj = context.active_object
        if obj and "lidar_attr_names" in obj:
            names = obj["lidar_attr_names"].split(",")
            return [(n, n, "") for n in names]
        return []
        
        
    def update_lidar_material(self, context):
        obj = context.active_object
        if obj and obj.type == 'MESH' and obj.get("lidar_attr_names"):
            attr_name = context.scene.lidar_selected_attr
            # Re-assign the material with the new attribute
            IMPORT_OT_las_data.assign_vertex_color_material(obj, attr_name)

    
    bpy.types.Scene.lidar_selected_attr = EnumProperty(
        name="LiDAR Attribute",
        description="Select attribute for coloring",
        items=get_lidar_attr_items,
        update=update_lidar_material
    )

class LIDAR_PT_InfoPanel(bpy.types.Panel):
    bl_label = "LiDAR Info"
    bl_idname = "LIDAR_PT_InfoPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'LiDAR'

    def draw(self, context):
        layout = self.layout

        # Retrieve custom properties from the object
        obj = context.active_object
        if obj and obj.get('lidar_filepath'):
            layout.prop(context.scene, "lidar_selected_attr", text="Color Attribute")
        else:
            layout.label(text="No LiDAR data available")


def menu_func_import(self, context):
    self.layout.operator(IMPORT_OT_las_data.bl_idname, text="LAS/LAZ data (.las, .laz)")

def register():
    bpy.utils.register_class(IMPORT_OT_las_data)
    bpy.utils.register_class(LIDAR_PT_InfoPanel)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)

def unregister():
    bpy.utils.unregister_class(IMPORT_OT_las_data)
    bpy.utils.unregister_class(LIDAR_PT_InfoPanel)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)

if __name__ == "__main__":
    register()