# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import omni.usd
import omni.client

from pxr import UsdGeom, Sdf, UsdPhysics, UsdShade


# Note: this script should be executed in Isaac Sim `Script Editor` window

def create_dofbot(asset_usd_path, dofbot_usd_path):
    # Duplicate dofbot.usd file
    omni.client.copy(asset_usd_path, dofbot_usd_path)

def create_dofbot_mesh(asset_usd_path, dofbot_mesh_usd_path):
    # Create dofbot_mesh.usd file
    omni.client.copy(asset_usd_path, dofbot_mesh_usd_path)
    omni.usd.get_context().open_stage(dofbot_mesh_usd_path)
    stage = omni.usd.get_context().get_stage()
    edits = Sdf.BatchNamespaceEdit()
    # Reparent joints in link5
    for d in ['Left', 'Right']:
        # Reparent finger 03 joint
        new_parent_path = f'/arm/link5/Finger_{d}_03'
        old_parent_path = f'{new_parent_path}/Finger_{d}_03'
        joint_path = f'{old_parent_path}/Finger_{d}_03_RevoluteJoint'
        edits.Add(Sdf.NamespaceEdit.Reparent(joint_path, new_parent_path, 0))
        # Reparent finger 02 joint
        new_parent_path = f'/arm/link5/Finger_{d}_02'
        old_parent_path = f'{new_parent_path}/Finger_{d}_02'
        joint_path = f'{old_parent_path}/Finger_{d}_02_RevoluteJoint'
        edits.Add(Sdf.NamespaceEdit.Reparent(joint_path, new_parent_path, 0))

    # Create parent Xforms
    # Joint 1 & 2 & 3
    reparent_tasks = [
        # base_link
        ['/arm/base_link/visuals', 'visuals_xform'],
        ['/arm/base_link/PCB_01', 'visuals_xform'],
        ['/arm/base_link/Base01_01', 'visuals_xform'],
        ['/arm/base_link/Antennas_01', 'visuals_xform'],
        ['/arm/base_link/collisions', 'collisions_xform'],
        # link1
        ['/arm/link1/visuals', 'visuals_xform'],
        ['/arm/link1/collisions', 'collisions_xform'],
        # link2
        ['/arm/link2/visuals', 'visuals_xform'],
        ['/arm/link2/collisions', 'collisions_xform'],
        # link3
        ['/arm/link3/visuals', 'visuals_xform'],
        ['/arm/link3/collisions', 'collisions_xform'],
        # link4
        ['/arm/link4/Wrist_Lift', 'geoms_xform'],
        # link5
        ['/arm/link5/Wrist_Twist/Wrist_Twist', 'geoms_xform'],
        ['/arm/link5/Finger_Left_01/Finger_Left_01', 'geoms_xform'],
        ['/arm/link5/Finger_Right_01/Finger_Right_01', 'geoms_xform'],
        ['/arm/link5/Finger_Left_03/Finger_Left_03', 'geoms_xform'],
        ['/arm/link5/Finger_Right_03/Finger_Right_03', 'geoms_xform'],
        ['/arm/link5/Finger_Left_02/Finger_Left_02', 'geoms_xform'],
        ['/arm/link5/Finger_Right_02/Finger_Right_02', 'geoms_xform'],
    ] # [prim_path, parent_xform_name]
    for task in reparent_tasks:
        prim_path, parent_xform_name = task
        old_parent_path = '/'.join(prim_path.split('/')[:-1])
        new_parent_path = f'{old_parent_path}/{parent_xform_name}'
        UsdGeom.Xform.Define(stage, new_parent_path)
        edits.Add(Sdf.NamespaceEdit.Reparent(prim_path, new_parent_path, -1))

    # Delete redundant materials
    edits.Add(Sdf.NamespaceEdit.Remove('/arm/link5/Looks'))
    stage.GetRootLayer().Apply(edits)

    # Fix link5 joints
    for d in ['Left', 'Right']:
        # finger 01 revolute joints
        joint_path = f'/arm/link5/Finger_{d}_01/Finger_{d}_01_RevoluteJoint'
        joint = UsdPhysics.Joint.Get(stage, joint_path)
        joint.GetBody1Rel().SetTargets(['/arm/link5/Wrist_Twist/geoms_xform/Wrist_Twist'])
        # finger 03 revolute joints
        joint_path = f'/arm/link5/Finger_{d}_03/Finger_{d}_03_RevoluteJoint'
        joint = UsdPhysics.Joint.Get(stage, joint_path)
        joint.GetBody0Rel().SetTargets([f'/arm/link5/Finger_{d}_03'])
        joint.GetBody1Rel().SetTargets([f'/arm/link5/Finger_{d}_01/geoms_xform/Finger_{d}_01'])
        # finger 02 spherical joints
        joint_path = f'/arm/link5/Finger_{d}_02/Finger_{d}_02_SphericalJoint'
        joint = UsdPhysics.Joint.Get(stage, joint_path)
        joint.GetBody0Rel().SetTargets([f'/arm/link5/Finger_{d}_03/geoms_xform/Finger_{d}_03'])
        joint.GetBody1Rel().SetTargets([f'/arm/link5/Finger_{d}_02/geoms_xform/Finger_{d}_02'])
        # finger 02 revolute joints
        joint_path = f'/arm/link5/Finger_{d}_02/Finger_{d}_02_RevoluteJoint'
        joint = UsdPhysics.Joint.Get(stage, joint_path)
        joint.GetBody0Rel().SetTargets([f'/arm/link5/Finger_{d}_02/geoms_xform/Finger_{d}_02'])
        joint.GetBody1Rel().SetTargets(['/arm/link5/Wrist_Twist/geoms_xform/Wrist_Twist'])

    for prim in stage.Traverse():
        if prim.GetTypeName() == 'Xform':
            # Copy Looks folder into visuals_xform and geoms_xform
            path = str(prim.GetPath())
            if path.endswith('visuals_xform') or path.endswith('geoms_xform'):
                omni.usd.duplicate_prim(stage, '/arm/Looks', f'{path}/Looks')
                ref = stage.GetPrimAtPath(f'{path}/Looks').GetReferences()
                ref.ClearReferences()
                ref.AddReference('./dofbot_materials.usd')
                pass
        elif prim.GetTypeName() == 'GeomSubset':
            # Bind GeomSubset to local materials
            path = str(prim.GetPath())
            parent_xform_path = path.split('/')
            while parent_xform_path[-1] != 'visuals_xform' and parent_xform_path[-1] != 'geoms_xform':
                parent_xform_path.pop()
            parent_xform_path = '/'.join(parent_xform_path)
            name = path.split('/')[-1]
            material = UsdShade.Material.Get(stage, f'{parent_xform_path}/Looks/{name}')
            UsdShade.MaterialBindingAPI(prim).Bind(material) # , UsdShade.Tokens.strongerThanDescendants)

    edits = Sdf.BatchNamespaceEdit()
    edits.Add(Sdf.NamespaceEdit.Remove('/arm/Looks'))
    stage.GetRootLayer().Apply(edits)

    # Save to file
    omni.usd.get_context().save_stage()

def create_dofbot_materials(asset_usd_path, dofbot_materials_usd_path):
    # Create dofbot_materials.usd file
    omni.client.copy(asset_usd_path, dofbot_materials_usd_path)
    omni.usd.get_context().open_stage(dofbot_materials_usd_path)
    stage = omni.usd.get_context().get_stage()
    edits = Sdf.BatchNamespaceEdit()
    # Extract Looks folder
    edits.Add(Sdf.NamespaceEdit.Reparent('/arm/Looks', '/', 0))
    # Remove everything else
    edits.Add(Sdf.NamespaceEdit.Remove('/World'))
    edits.Add(Sdf.NamespaceEdit.Remove('/arm'))
    # Apply & save to file
    stage.GetRootLayer().Apply(edits)
    prim = stage.GetPrimAtPath('/Looks')
    stage.SetDefaultPrim(prim)
    omni.usd.get_context().save_stage()

def create_dofbot_instanceable(dofbot_mesh_usd_path, dofbot_instanceable_usd_path):
    omni.client.copy(dofbot_mesh_usd_path, dofbot_instanceable_usd_path)
    omni.usd.get_context().open_stage(dofbot_instanceable_usd_path)
    stage = omni.usd.get_context().get_stage()
    # Set up references and instanceables
    for prim in stage.Traverse():
        if prim.GetTypeName() != 'Xform':
            continue
        # Add reference to visuals_xform, collisions_xform, geoms_xform, and make them instanceable
        path = str(prim.GetPath())
        if path.endswith('visuals_xform') or path.endswith('collisions_xform') or path.endswith('geoms_xform'):
            ref = prim.GetReferences()
            ref.ClearReferences()
            ref.AddReference('./dofbot_mesh.usd', path)
            prim.SetInstanceable(True)
    # Save to file
    omni.usd.get_context().save_stage()

def create_block_indicator():
    asset_usd_path = 'omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Props/Blocks/block.usd'
    block_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Props/Blocks/block.usd'
    omni.client.copy(asset_usd_path, block_usd_path)
    omni.usd.get_context().open_stage(block_usd_path)
    stage = omni.usd.get_context().get_stage()
    edits = Sdf.BatchNamespaceEdit()
    edits.Add(Sdf.NamespaceEdit.Remove('/object/object/collisions'))
    stage.GetRootLayer().Apply(edits)
    omni.usd.get_context().save_stage()

    asset_usd_path = 'omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Props/Blocks/block_instanceable.usd'
    block_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Props/Blocks/block_instanceable.usd'
    omni.client.copy(asset_usd_path, block_usd_path)
    omni.usd.get_context().open_stage(block_usd_path)
    stage = omni.usd.get_context().get_stage()
    edits = Sdf.BatchNamespaceEdit()
    edits.Add(Sdf.NamespaceEdit.Remove('/object/object/collisions'))
    stage.GetRootLayer().Apply(edits)
    omni.usd.get_context().save_stage()

if __name__ == '__main__':
    asset_usd_path = 'omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Robots/Dofbot/dofbot.usd'
    dofbot_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Robots/Dofbot/dofbot.usd'
    dofbot_materials_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Robots/Dofbot/dofbot_materials.usd'
    dofbot_mesh_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Robots/Dofbot/dofbot_mesh.usd'
    dofbot_instanceable_usd_path = 'omniverse://localhost/Projects/J3soon/Isaac/2022.1/Isaac/Robots/Dofbot/dofbot_instanceable.usd'
    create_dofbot(asset_usd_path, dofbot_usd_path)
    create_dofbot_materials(asset_usd_path, dofbot_materials_usd_path)
    create_dofbot_mesh(asset_usd_path, dofbot_mesh_usd_path)
    create_dofbot_instanceable(dofbot_mesh_usd_path, dofbot_instanceable_usd_path)
    create_block_indicator()
    print("Done!")
