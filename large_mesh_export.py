import bpy
import array
import sys

class AABB:
    def __init__(self, vectorInit = None):
        self.min = [0,0,0]
        self.max = [0,0,0]

        if vectorInit:
            self.min = list(vectorInit)
            self.max = list(vectorInit)

    def encase(self, v):
        self.min[0] = min(self.min[0], v[0])
        self.min[1] = min(self.min[1], v[1])
        self.min[2] = min(self.min[2], v[2])

        self.max[0] = max(self.max[0], v[0])
        self.max[1] = max(self.max[1], v[1])
        self.max[2] = max(self.max[2], v[2])

    # find largest axis to split
    # 0=x, 1=y, 2=z
    def findSplittingAxis(self):
        xlen = self.max[0] - self.min[0]
        ylen = self.max[1] - self.min[1]
        zlen = self.max[2] - self.min[2]

        if xlen > ylen and xlen > zlen:
            return 0
        elif ylen > xlen and ylen > zlen:
            return 1
        else:
            return 2

class KDTreeNode:
    def __init__(self, id=0, maxDepth=10, maxTris=10000):
        self._id = id
        # max tris before split
        self._maxTris = maxTris
        # max tree depth by default?
        self._maxDepth = maxDepth
        # no parent
        self.parent = None
        # no children
        self.children = [None, None]
        # aabb 
        self.aabb = AABB()
        # default split is x axis
        self.axisId = 0 
        # triangle data (LOOP TRIANGLES TO BE EXACT, from BLENDER MESH DATA)
        self.tris = []
        self.trisCenters = []
        # current depth?
        self.depth = 0
        # which mesh object do we contain? -1 is invalid
        self.object_id = -1

    def isRoot(self):
        return self.parent == None

    def isLeaf(self):
        return self.children[0] == self.children[1] and self.children[0] == None

    # split ourselves, and re-store data to our children
    def split(self):
        # ARE WE TOO DEEP? if so, do nothing
        if self.depth >= self._maxDepth:
            return

        # okay, gotta spawn two children?
        self.children[0] = KDTreeNode(self._id * 2 + 1, self._maxDepth, self._maxTris)
        self.children[1] = KDTreeNode(self._id * 2 + 2, self._maxDepth, self._maxTris)
        # compute the splitting axis
        self.axisId = self.aabb.findSplittingAxis()

        c1 = self.children[0]
        c2 = self.children[1]
        c1.parent = self
        c2.parent = self
        # copy parent aabb
        c1.aabb.min = list(self.aabb.min)
        c1.aabb.max = list(self.aabb.max)
        c2.aabb.min = list(self.aabb.min)
        c2.aabb.max = list(self.aabb.max)
        # set depth as depth + 1
        c1.depth = self.depth + 1
        c2.depth = self.depth + 1

        if self.axisId >= 0 and self.axisId <= 2:
            # split in whatever axis, so modify the x extent
            aId = self.axisId
            halfExt = 0.5 * (self.aabb.min[aId] + self.aabb.max[aId])
            c1.aabb.max[aId] = halfExt
            c2.aabb.min[aId] = halfExt

        # now empty our triangles into our children?
        for i in range(len(self.tris)):
            t = self.tris[i]
            tCenter = self.trisCenters[i]
            self.addTriangle(tCenter, t)
        
        # clear data
        self.tris = []
        self.trisCenters = []

    # logic to add triangle indices data?
    # vCenter = triangle center?
    # indices = an array of 3 indices
    def addTriangle(self, vCenter, indices):
        if self.isLeaf():
            # we're leaf nodes, just add it?
            self.trisCenters.append(vCenter)
            self.tris.append(indices)

            # are we over limit?
            if len(self.tris) > self._maxTris:
                self.split()
        else:
            # gotta recurse to child?
            # which child it is then?
            aId = self.axisId
            cId = 0
            if vCenter[aId] > (0.5 * self.aabb.min[aId] + self.aabb.max[aId]):
                cId = 1
            # got child id, recurse to em
            self.children[cId].addTriangle(vCenter, indices)

    # debug print?
    def debugPrint(self):
        print("Node[%d] @ depth(%d): tris(%d), aabb(%.4f %.4f %.4f | %.4f %.4f %.4f), leaf?(%s) --> mesh_id(%d)" % (
            self._id, self.depth, len(self.tris), 
            self.aabb.min[0], self.aabb.min[1], self.aabb.min[2],
            self.aabb.max[0], self.aabb.max[1], self.aabb.max[2],
            ('False', 'True')[self.isLeaf()], self.object_id
        ))
        # recurse if we have children
        if not self.isLeaf():
            print(" -- Children: { %d, %d }" % (self.children[0]._id, self.children[1]._id))
            self.children[0].debugPrint()
            self.children[1].debugPrint()

# count nodes and renumber them
def nodeCount(tree, renumber=True):
    # use iterative method
    stack = [tree]
    count = 0

    while len(stack):
        tr = stack.pop()

        if renumber:
            tr._id = count

        count += 1
        if not tr.isLeaf():
            stack.append(tr.children[0])
            stack.append(tr.children[1])

    return count


# collect good leaves only, and set its object id
def collectGoodLeaves(tree):
    # use iterative method
    stack = [tree]

    goodOnes = []

    # while stack not empty
    while len(stack):
        # pop one from stack
        tr = stack.pop()
        # is this one leaf?
        if tr.isLeaf():
            # do real check
            if len(tr.tris):
                # we do have data
                goodOnes.append(tr)
        else:
            # just push children to stack
            stack.append(tr.children[0])
            stack.append(tr.children[1])
    # by now, the good ones must have been filled. 
    # now, we renumber em
    for (idx, leaf) in enumerate(goodOnes):
        leaf.object_id = idx
    return goodOnes

# helper to make a kdtree out of a mesh object?
def buildTree(obj, maxDepth=10, maxTris=10000):
    print("Building KDTree from object: %s, with max depth: %d, max tris: %d" % (obj.name, maxDepth, maxTris))
    # mesh = bpy.types.Mesh()
    mesh = obj.data

    # loop over all triangles?
    mesh.calc_loop_triangles()
    mesh.calc_normals_split()
    mesh.calc_tangents()

    print("Computing AABB from %d vertices..." % len(mesh.vertices))

    # spawn the root node
    root = KDTreeNode(0, maxDepth=maxDepth, maxTris=maxTris)

    # compute AABB
    for (idx, vert) in enumerate(mesh.vertices):
        v = [vert.co.x, -vert.co.z, vert.co.y]

        if (idx == 0):
            # first vector, set to it
            root.aabb = AABB(v)
        else:
            # try to encase
            root.aabb.encase(v)

    # now loop all over triangles, compute its center and add them to our node?
    for (idx, t) in enumerate(mesh.loop_triangles):
        # copy indices
        t_idx = list(t.vertices)
        # compute centers
        t_center = [0, 0, 0]
        for i in range(3):
            l = mesh.loops[t_idx[i]]
            v = mesh.vertices[l.vertex_index]
            t_center[0] += v.co[0]
            t_center[1] += -v.co[2]
            t_center[2] += v.co[1]
        
        t_center[0] /= 3.0
        t_center[1] /= 3.0
        t_center[2] /= 3.0

        # now add em? ADD THE TRIANGLE DIRECTLY!!!!
        root.addTriangle(t_center, t)

    # return the root node, but renumber them first
    nodeCount(root)
    return root
# 

bl_info = {
    "name": "Bowie Large Mesh Format Exporter",
    "author": "Bowie",
    "blender": (2, 83, 0),
    "version": (0, 0, 1),
    "location": "File > Import-Export",
    "description": "Export Large Mesh File (LMF) containing mesh data and its kdTree",
    "category": "Import-Export"
}


"""
Author: Bowie
This exporter defines a pretty basic export for
OGL compatible vertex buffer

Vertex Format (using bit position to toggle availability):
(1 << 0) : POSITION
(1 << 1) : NORMAL
(1 << 2) : UV0
(1 << 3) : TANGENT + BITANGENT
(1 << 4) : UV1 (NOT IMPLEMENTED YET)
(1 << 5) : COLOR (NOT IMPLEMENTED YET)
(1 << 6) : BONE_WEIGHTS + IDS (NOT IMPLEMENTED YET)
(1 << 7) : TWEEN (NOT IMPLEMENTED YET)
"""
VTF_POS     = (1<<0)
VTF_NORMAL  = (1<<1)
VTF_UV0     = (1<<2)
VTF_TANGENT_BITANGENT      = (1<<3)
VTF_UV1     = (1<<4)
VTF_COLOR   = (1<<5)
VTF_BONE_DATA   = (1<<6)
VTF_TWEEN   = (1<<7)

VTF_DEFAULT = VTF_POS | VTF_NORMAL | VTF_UV0


# helper to make binary buffer
def make_buffer(format, data):
    buf = array.array(format, data)
    if sys.byteorder != 'little':
        buf.byteswap()
    return buf

# compute bytes per vertex
def bytesPerVertex(vtx_format):
    totalSize = 0
    if vtx_format & VTF_POS: totalSize += 12
    if vtx_format & VTF_NORMAL: totalSize += 12
    if vtx_format & VTF_UV0: totalSize += 8
    if vtx_format & VTF_TANGENT_BITANGENT: totalSize += 24
    if vtx_format & VTF_UV1: totalSize += 8
    if vtx_format & VTF_COLOR: totalSize += 12
    if vtx_format & VTF_BONE_DATA: totalSize += 20

    return totalSize
##

# build mesh data from a single leafnode
def buildSingleMeshFromLeafNode(obj, node, format=VTF_DEFAULT):
    if not node.isLeaf():
        return None
    
    # safe to go
    mesh = obj.data
    verts = mesh.vertices   # xyz 
    loops = mesh.loops  # normal, tangent
    uv0 = mesh.uv_layers[0].data    # uv0
    uv1 = None  # uv1
    if (len(mesh.uv_layers) > 1):
        uv1 = mesh.uv_layers[1].data
    
    # init submeshes bucket?
    submeshes = []
    for i in range(len(mesh.materials)):
        submeshes.append([])
    
    # to store our unique vertices
    unique_verts = []
    # now loop over our triangle loops
    n = node

    # for each triangleloop
    for (t_id, t) in enumerate(n.tris):
        newTriangle = []
        # for each vertex on it
        for i in range(3):
            # compute vertex?
            newVertex = []

            if format & VTF_POS:
                rp = verts[t.vertices[i]].co
                pos = [rp.x, rp.z, -rp.y]
                newVertex.append(pos)
            
            if format & VTF_NORMAL:
                rn = loops[t.loops[i]].normal
                normal = [rn.x, rn.z, -rn.y]
                newVertex.append(normal)

            if format & VTF_UV0:
                uv = uv0[t.loops[i]].uv
                newVertex.append([uv.x, uv.y])

            if format & VTF_TANGENT_BITANGENT:
                tgt = loops[t.loops[i]].tangent
                btg = loops[t.loops[i]].bitangent
                newVertex.append([
                    tgt.x, tgt.z, -tgt.y,
                    btg.x, btg.z, -btg.y
                ])

            if format & VTF_UV1:
                # if we have no uv1 data, raise an exception
                if not uv1:
                    raise Exception("Fuck it no uv1 for this mesh, even if it's requested!")

                uv = uv1[t.loops[i]].uv
                newVertex.append([uv.x, uv.y])

            # we got a vertex now, to lookup its index
            if newVertex not in unique_verts:
                unique_verts.append(newVertex)

            newTriangle.append(unique_verts.index(newVertex))
        
        # got a new triangle, now append to submesh?
        submeshes[t.material_index].append(newTriangle)

    # return a tuple of a vertices and submeshes?
    return (unique_verts, submeshes)

# build meshes from tree
def buildMeshesFromTree(obj, tree, format=VTF_DEFAULT):
    leaves = collectGoodLeaves(tree)
    meshes = []
    for (idx, l) in enumerate(leaves):
        m = buildSingleMeshFromLeafNode(obj, l, format)
        meshes.append(m)
        l.object_id = idx
    return meshes

###
# buildBuffers: return tuple of vb and ib
def buildBuffers(obj, report=None, format=VTF_DEFAULT, bounding=None):
    
    # mesh
    m = obj.data
    
    # start
    print("BCF_START_BUILDING_BUFFERS...\n")
    # compute data
    print("BCF_COMPUTE_TRIANGLES...\n")
    m.calc_loop_triangles()

    print("BCF_COMPUTE_SPLIT_NORMALS...\n")
    m.calc_normals_split()

    print("BCF_COMPUTE_TANGENTS...\n")
    m.calc_tangents()
    
    # now we access the data
    # the mesh loops (for v_idx, normal, tangent, bitangent)
    mls = m.loops
    print("BCF_DISCOVERED: Loops(%d)\n" % len(mls))
    # vertex list
    verts = m.vertices
    print("BCF_DISCOVERED: Vertices(%d)\n" % len(verts))
    # triangle list
    tris = m.loop_triangles
    print("BCF_DISCOVERED: Triangles(%d)\n" % len(tris))
    # uv0 data
    uv0_data = m.uv_layers[0].data
    print("BCF_DISCOVERED: UV0Coords(%d)\n" % len(uv0_data))

    # uv1 data (if available)
    uv1_data = None
    if len(m.uv_layers) > 1:
        uv1_data = m.uv_layers[1].data
        print("BCF_DISCOVERED: UV1Coords(%d)\n" % len(uv1_data))
    else:
        print("BCF_DISCOVERED: No UV1Coords available\n")

    # submeshes = materials
    submeshes_count = len(m.materials)
    print("BCF_SUBMESHES_COUNT: %d" % submeshes_count)

    
    # empty list, fill later
    unique_verts = []

    # bounding box
    boundingSet = False
    
    # real triangle data (optimized)
    # real_tris = []
    # allocate submeshes data
    submeshes_data = []
    for i in range(submeshes_count):
        submeshes_data.append({
            "material": m.materials[i].name,
            "data": []
        })
    
    # for each triangle
    print("BCF_START_PROCESSING_TRIANGLES...\n")
    for t in tris:
        triangle = [] # only list of verts
        
        # let's loop over indices in this triangle
        # REVISED (NOPE, BLENDER IS ALREADY FRONT = CCW)
        for i in range(3):
            # get loop id
            loop_id = t.loops[i]
            # get loop data
            vtx = mls[loop_id]

            # Transform by -90 along x axis
            # to conform to opengl standard
            # [x y z] => [x -z y]
            
            # build vertex data as flat array
            '''
            [pos.x, -pos.z, pos.y],
            [normal.x, -normal.z, normal.y],
            [uv.x, uv.y]
            '''
            vdata = []

            # grab vertex data (only when needed)
            # ALL VERTEX DATA NEED TO BE ROTATED 90deg
            # ALONG X AXIS. JUST SWAP Y <-> Z
            # position
            rawPos = verts[vtx.vertex_index].co
            pos = [rawPos.x, rawPos.z, -rawPos.y]

            # update bounding if provided
            if bounding:
                # ALONG X AXIS
                if pos[0] < bounding[0] or not boundingSet:
                    bounding[0] = pos[0]
                if pos[0] > bounding[3] or not boundingSet:
                    bounding[3] = pos[0]
                # ALONG Y AXIS
                if pos[1] < bounding[1] or not boundingSet:
                    bounding[1] = pos[1]
                if pos[1] > bounding[4] or not boundingSet:
                    bounding[4] = pos[1]
                # ALONG Z AXIS
                if pos[2] < bounding[2] or not boundingSet:
                    bounding[2] = pos[2]
                if pos[2] > bounding[5] or not boundingSet:
                    bounding[5] = pos[2]

                boundingSet = True

            # position, use above data
            if format & VTF_POS:
                # pos = verts[vtx.vertex_index].co
                vdata.append(pos)

            # normal
            if format & VTF_NORMAL:
                normal = vtx.normal
                vdata.append([normal.x, normal.z, -normal.y])

            # uv0
            if format & VTF_UV0:
                uv = uv0_data[loop_id].uv
                vdata.append([uv.x, uv.y])

            # tangent + bitangent
            if format & VTF_TANGENT_BITANGENT:
                tangent = vtx.tangent
                bitangent = vtx.bitangent
                vdata.append([
                    tangent.x, tangent.z, -tangent.y,
                    bitangent.x, bitangent.z, -bitangent.y
                ])

            # uv1
            if format & VTF_UV1 and uv1_data != None:
                uv = uv1_data[loop_id].uv
                vdata.append([uv.x, uv.y])
            
            # add if no vertex found, add as new
            # otherwise, this vertex is not unique
            if vdata not in unique_verts:
                unique_verts.append(vdata)
                
            # grab its index
            # get unique id of such vertex
            unique_v_idx = unique_verts.index(vdata)
            # add to triangle data
            triangle.append(unique_v_idx)
        
        # append real tris data
        # real_tris.append(triangle)
        # append to appropriate submeshes?
        submeshes_data[t.material_index]['data'].append(triangle)

    print("BCF_BUFFERS_BUILT\n")
    return (unique_verts, submeshes_data)

# preparation step
def can_write(context, vtx_format, me):
    print("CHECKING IF CONTEXT MAKES SENSE...\n")
    
    # check some requirements...
    # make sure an object is selected
    if (len(context.selected_objects) == 0):
        print("NO OBJECT SELECTED. BAIL....")
        me.report({'ERROR'}, 'NO OBJECT SELECTED!')
        return False
    
    # make sure it's a mesh object
    obj = context.selected_objects[0]
    m = obj.data

    if type(m) != bpy.types.Mesh:
        print("OBJECT DATA IS NOT MESH!")
        me.report({'ERROR'}, "SELECTED OBJECT '%s' IS NOT A MESH" % obj.name)
        return False

    # make sure it has at least 1 uv map
    if (len(m.uv_layers) < 1):
        print("NO UV MAP FOUND!")
        me.report({'ERROR'}, "AT LEAST 1 UV MAP MUST BE CREATED!")
        return False

    # if uv1 is specified, make sure it has 2 uv maps
    if (vtx_format & VTF_UV1) and len(m.uv_layers) < 2:
        print("CANNOT FIND SECOND UV MAP!! MAYBE EXPORT WITH UV0 only!")
        me.report({'ERROR'}, "UV1 WAS REQUESTED BUT THERE WAS ONLY 1 UV MAP!")
        return False

    return True

def write_tree_to_ascii(filepath, obj, tree, meshes, vtx_format):
    f = open(filepath, mode="w", encoding="utf-8")

    # first, write object name?
    f.write("name: %s\n" % (obj.name))
    # node count?
    f.write("node_count: %d\n" % (nodeCount(tree, False)))
    # mesh data?
    f.write("mesh_obj_count: %d\n" % (len(meshes)))
    # submesh count per mesh
    f.write("submesh_per_obj: %d\n" % (len(obj.data.materials)))
    # now write node data first?
    # id, parent_id, bounding_box, mesh_id
    # use iterative method
    stack = [tree]

    while len(stack):
        node = stack.pop()

        parent_id = -1
        if node.parent:
            parent_id = node.parent._id
        self_id = node._id
        aabb = node.aabb
        mesh_id = node.object_id

        # write it?
        str = "node: id(%d), parent_id(%d), aabb(%.4f %.4f %.4f | %.4f %.4f %.4f) mesh_id(%d)\n" % (
            self_id, parent_id, aabb.min[0], aabb.min[1], aabb.min[2], aabb.max[0], aabb.max[1], aabb.max[2],
            mesh_id
        )

        f.write(str)

        # if has children, add more
        if not node.isLeaf():
            stack.append(node.children[0])
            stack.append(node.children[1])
    # now we write mesh data!
    for (m_id, m) in enumerate(meshes):
        # write down unique vertices?
        # m = (unique_verts[], submesh[])
        str = "mesh[%d]: vert(%d) submesh(%d)\n" % (m_id, len(m[0]), len(m[1]))
        f.write(str)
        # write down vertices
        for (v_id, v) in enumerate(m[0]):
            str = "v[%d]:" % v_id
            idx = 0
            if vtx_format & VTF_POS:
                str += " pos(%.4f %.4f %.4f)" % (v[idx][0], v[idx][1], v[idx][2])
                idx += 1
            if vtx_format & VTF_NORMAL:
                str += " norm(%.4f %.4f %.4f)" % (v[idx][0], v[idx][1], v[idx][2])
                idx += 1
            if vtx_format & VTF_UV0:
                str += " uv0(%.4f %.4f)" % (v[idx][0], v[idx][1])
                idx += 1
            if vtx_format & VTF_TANGENT_BITANGENT:
                str += " tb(%.4f %.4f %.4f | %.4f %.4f %.4f)" % (
                    v[idx][0], v[idx][1], v[idx][2],
                    v[idx][3], v[idx][4], v[idx][5]
                )
                idx += 1
            if vtx_format & VTF_UV1:
                str += " uv1(%.4f %.4f)" % (v[idx][0], v[idx][1])
            
            str += "\n"
            f.write(str)
        # write submeshes data
        for (sm_id, sm) in enumerate(m[1]):
            f.write("-submesh[%d]: tris(%d)\n" % (sm_id, len(sm)))
            for (t_id, t) in enumerate(sm):
                f.write("--tri[%d]: %d %d %d\n" % (t_id, t[0], t[1], t[2]))

    f.close()



def do_write_tree(context, filepath, vtx_format, me, maxDepth, maxTris, mode="ascii"):
    print("LMF_EXPORT_STARTED...")

    if not can_write(context, vtx_format, me):
        return {'CANCELLED'}

    obj = context.selected_objects[0]

    print("WRITING(%s), VTX_FORMAT(%d), MAX_DEPTH(%d), MAX_TRIS(%d), MODE(%s)" % (
        filepath, vtx_format, maxDepth, maxTris, mode
    ))
    # build tree
    tree = buildTree(obj, maxDepth=maxDepth, maxTris=maxTris)
    meshes = buildMeshesFromTree(obj, tree, vtx_format)

    write_tree_to_ascii(filepath, obj, tree, meshes, vtx_format)

    me.report({'INFO'}, "LARGE MESH FILE WRITTEN!")

    return {'FINISHED'}


# do the writing (easy)
def do_write(context, filepath, vtx_format, me, mode="ascii"):
    print("BCF_EXPORT_STARTED...\n")

    # check some requirements...
    if not can_write(context, vtx_format, me):
        return {'CANCELLED'}
    
    # grab object
    obj = context.selected_objects[0]

    # process the object
    print("BCF_VERTEX_FORMAT: %d\n" % vtx_format)
    boundingData = [0,0,0,0,0,0]
    vb, ibs = buildBuffers(obj, report=me.report,format=vtx_format, bounding=boundingData)

    print("AABB: min(%.4f, %.4f, %.4f) max(%.4f, %.4f, %.4f)\n" % (boundingData[0], boundingData[1], boundingData[2], boundingData[3], boundingData[4], boundingData[5]) )

    if mode == 'ascii':
        # write to ascii for now (changeable later)
        total_tris = write_to_ascii(filepath, vb, ibs, vtx_format, obj.name, boundingData)
    elif mode == 'binary':
        total_tris = write_to_binary(filepath, vb, ibs, vtx_format, obj.name, boundingData)
    else:
        # error happens
        me.report({'INFO'}, "UNKNOWN WRITE TYPE '%s'" % mode)
        return {'CANCELLED'}

    me.report({'INFO'}, "Done writing shits: %d unique vertices and %d submeshes, totaling %d tris " % (len(vb), len(ibs), total_tris))

    return {'FINISHED'}

# write_to_binary, write binary file
# FORMAT IS AS FOLLOWS:
# 1b : vtx_format
# 1b : bytes_per_vertex
# 2b : vertex_count (max 65535 vertex)
# 4b : vertex_buffer_size_in_bytes
# 2b : sub_mesh_count
# 32b: objname
# 2b : total_tris
# 24b: bounding_box (min: 3 float, max: 3 float)
# --SUB_MESH_DATA_---
# { 32b: material_name, 2b: begin_at, 2b: total_tri }
# { vertex_buffer }
# { index_buffer }
def write_to_binary(filepath, vb, ibs, vtx_format, objname, bounding):
    # preprocess indices data first
    total_tris = 0
    for ib in ibs:
        ib['begin_at'] = total_tris
        ib['total_elem'] = len(ib['data'])
        total_tris += ib['total_elem']

    print("BCF_BINARY_WRITE: (%s)...\n" % filepath)
    vertex_size = bytesPerVertex(vtx_format)

    print("format: %d, bytes per vertex: %d\n" % (vtx_format, vertex_size) )
    f = open(filepath, 'wb')

    # write vtx format and bytes per vertex
    # 1b : vtx_format
    # 1b : bytes_per_vertex
    print("BCF_WRITE_HEADER...\n")
    buf = make_buffer('B',[ vtx_format, vertex_size ])
    f.write(buf)
    
    # 2b : vertex_count (max 65535 vertex)
    # 4b : vertex_buffer_size_in_bytes
    vertex_count = len(vb)
    '''
    'b'         signed integer     1
    'B'         unsigned integer   1
    'u'         Unicode character  2 (see note)
    'h'         signed integer     2
    'H'         unsigned integer   2
    'i'         signed integer     2
    'I'         unsigned integer   2
    'l'         signed integer     4
    'L'         unsigned integer   4
    'q'         signed integer     8 (see note)
    'Q'         unsigned integer   8 (see note)
    'f'         floating point     4
    'd'         floating point     8
    '''
    f.write(make_buffer('H', [vertex_count]))
    vertex_buffer_size_in_bytes = vertex_count * vertex_size
    print("vertex_count: %d, vertex_buffer_size_in_bytes: %d\n" % ( vertex_count, vertex_buffer_size_in_bytes ))
    f.write(make_buffer('L', [vertex_buffer_size_in_bytes]))

    # 2b : sub_mesh_count
    submesh_count = len(ibs)
    print("submesh_count: %d\n" % submesh_count)
    f.write(make_buffer('H', [submesh_count]))

    # 32b: objname
    print("objname: %s\n" % objname)
    buf = bytearray(objname, 'utf-8')
    padded_buf = buf.ljust(32, b'\0')
    f.write(padded_buf)

    # 2b: total_tris
    print("total_tris: %d\n" % total_tris)
    buf = make_buffer('H', [total_tris])
    f.write(buf)

    # 24b: bounding_box
    print("bounding_box: %.4f %.4f %.4f %.4f %.4f %.4f\n" % (bounding[0], bounding[1], bounding[2], bounding[3], bounding[4], bounding[5]))
    buf = make_buffer('f', bounding)
    f.write(buf)

    # SUBMESH_DATA
    for ib in ibs:
        print("writing mesh(%s, begin: %d, total: %d)\n" % (ib['material'], ib['begin_at'], ib['total_elem']))

        # 32b: material_name
        buf = bytearray(ib['material'], 'utf-8')
        padded_buf = buf.ljust(32, b'\0')
        f.write(padded_buf)

        # 2b: begin_at
        buf = make_buffer('H', [ib['begin_at']])
        f.write(buf)
        
        # 2b: total_elem
        buf = make_buffer('H', [ib['total_elem']])
        f.write(buf)

    # VERTEX_BUFFER
    print("writing vertex buffer...\n")
    for v_idx, v in enumerate(vb):
        i = 0   # track data pointer
        # write position if there is
        if vtx_format & VTF_POS:
            buf = make_buffer('f', v[i])
            f.write(buf)
            i+=1

        # write normal
        if vtx_format & VTF_NORMAL:
            buf = make_buffer('f', v[i])
            f.write(buf)
            i+=1

        # write uv0
        if vtx_format & VTF_UV0:
            buf = make_buffer('f', v[i])
            f.write(buf)
            i+=1

        # write tangent + bitangent
        if vtx_format & VTF_TANGENT_BITANGENT:
            buf = make_buffer('f', v[i])
            f.write(buf)
            i+=1

        # write uv1
        if vtx_format & VTF_UV1:
            buf = make_buffer('f', v[i])
            f.write(buf)
            i+=1

    
    # WRITE ALL INDICES
    # for each submesh
    for ib in ibs:
        for t in ib['data']:
            f.write(make_buffer('H', t))

    f.close()

    return total_tris

def write_to_ascii(filepath, vb, ibs, vtx_format, objname, bounding):

    # now write the data
    print("BCF_WRITING_TO_FILE: (%s)...\n" % filepath)
    f = open(filepath, 'w', encoding='utf-8')
    f.write("vtx_format: %d\n" % vtx_format)
    f.write("object: %s\n" % objname)

    # write vertex count and data
    print("BCF_WRITING_VERTEX_DATA...\n")
    f.write("vertex_count: %d\n" % len(vb))
    f.write("bounding_box: %.4f %.4f %.4f %.4f %.4f %.4f\n" % (bounding[0], bounding[1], bounding[2], bounding[3], bounding[4], bounding[5]))
    for v_idx, v in enumerate(vb):
        # print id
        f.write("%d:" % v_idx)
        # conditional write here....
        i = 0   # track the data pointer
        if vtx_format & VTF_POS: 
            f.write("\tv(%.4f %.4f %.4f)" % (v[i][0], v[i][1], v[i][2]))
            i+=1

        if vtx_format & VTF_NORMAL: 
            f.write("\tn(%.4f %.4f %.4f)" % (v[i][0], v[i][1], v[i][2]))
            i+=1

        if vtx_format & VTF_UV0: 
            f.write("\tu0(%.4f %.4f)" % (v[i][0], v[i][1]))
            i+=1

        if vtx_format & VTF_TANGENT_BITANGENT: 
            f.write("\ttb(%.4f %.4f %.4f %.4f %.4f %.4f)" % (v[i][0], v[i][1], v[i][2], v[i][3], v[i][4], v[i][5]))
            i+=1

        if vtx_format & VTF_UV1: 
            f.write("\tu1(%.4f %.4f)" % (v[i][0], v[i][1]))
            i+=1

        f.write("\n")
    print("DONE.\n")

    # write index data
    print("BCF_WRITING_TRIANGLE_DATA...\n")
    f.write("submeshes_count: %d\n" % len(ibs))
    # for each submeshes
    total_tris = 0
    for ib in ibs:
        f.write("material: %s\n" % ib['material'])
        f.write("triangle_count: %d\n" % len(ib['data']))
        total_tris += len(ib['data'])
        for t_idx, t in enumerate(ib['data']):
            f.write("%d: %d %d %d\n" % (t_idx, t[0], t[1], t[2]))
    
    f.flush()
    f.close()
    print("DONE.\n")

    # return length of vb, num of submeshes, and total tris
    return total_tris

# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import Operator


class LMFExporter(Operator, ExportHelper):
    """Export to Large Mesh Format (LMF)"""
    bl_idname = "lmf_exporter.export"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "EXPORT LARGE MESH!"

    # ExportHelper mixin class uses this
    filename_ext = ".lmf"

    filter_glob: StringProperty(
        default="*.lmf",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # some default vertex format
    vertex_has_pos: BoolProperty(name="Position", description="XYZ vertex data", default=(VTF_DEFAULT & VTF_POS)!=0)
    vertex_has_normal: BoolProperty(name="Normal", description="XYZ normal data", default=(VTF_DEFAULT & VTF_NORMAL)!=0)
    vertex_has_uv0: BoolProperty(name="UV0", description="primary (first) UV", default=(VTF_DEFAULT & VTF_UV0)!=0)
    vertex_has_tangents: BoolProperty(name="Tangent+Bitangent", description="tangent+bitangent 2x(XYZ)", default=(VTF_DEFAULT & VTF_TANGENT_BITANGENT)!=0)
    vertex_has_uv1: BoolProperty(name="UV1", description="secondary UV", default=(VTF_DEFAULT & VTF_UV1)!=0)
    vertex_has_color: BoolProperty(name="Color", description="(RGB) vertex color", default=(VTF_DEFAULT & VTF_COLOR)!=0)
    vertex_has_bone: BoolProperty(name="Bone Weights+IDs", description="Bone Weights + ID for skeletal animation", default=(VTF_DEFAULT & VTF_BONE_DATA)!=0)
    vertex_has_tween: BoolProperty(name="Tween", description="XYZ vertex animation data", default=(VTF_DEFAULT & VTF_TWEEN)!=0)

    max_depth: IntProperty(name="Max Tree Depth", description="Maximum depth of the KD Tree", default=10, min=4, max=32)
    max_tris: IntProperty(name="Max Triangles", description="Maximum triangle before splitting", default=1000, min=40)

    write_mode: EnumProperty(
        items=(
            ('ascii', "ASCII", "Human readable format"),
            ('binary', "Binary", "Compact memory size")
        ),
        name="File Type",
        description="What kind of file output to write",
        default='ascii'
    )

    def execute(self, context):
        # build a vertex format before executing
        format = 0
        if self.vertex_has_pos: format |= VTF_POS
        if self.vertex_has_normal: format |= VTF_NORMAL
        if self.vertex_has_uv0: format |= VTF_UV0
        if self.vertex_has_tangents: format |= VTF_TANGENT_BITANGENT
        if self.vertex_has_uv1: format |= VTF_UV1
        if self.vertex_has_color: format |= VTF_COLOR
        if self.vertex_has_bone: format |= VTF_BONE_DATA
        if self.vertex_has_tween: format |= VTF_TWEEN


        # return do_write(context, self.filepath, format, self, self.write_mode)
        return do_write_tree(context, self.filepath, format, self, self.max_depth, self.max_tris)


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(LMFExporter.bl_idname, text="LMF Export")


def register():
    bpy.utils.register_class(LMFExporter)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(LMFExporter)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.lmf_exporter.export('INVOKE_DEFAULT')

# def dump(obj):
#    for attr in dir(obj):
#        if hasattr( obj, attr ):
#            print( "obj.%s = %s" % (attr, getattr(obj, attr)))

# o = bpy.context.selected_objects[0]
# t = buildTree(o, 10, 1000)
# meshes = buildMeshesFromTree(o, t)

# t.debugPrint()

# for (idx, m) in enumerate(meshes):
#     print("mesh[%d]: verts(%d), submesh(%d)" % (idx, len(m[0]), len(m[1])))
#     # dump every vertices? nope, just the triangle for now I guess
#     for (si, sm) in enumerate(m[1]):
#         print("-- submesh[%d]: tris(%d)" % (si, len(sm)))
#         for (ti, t) in enumerate(sm):
#             print("  -- t(%d %d %d)" % (t[0], t[1], t[2]))
# print("nodes count: %d" % (nodeCount(t)))

# gl = collectGoodLeaves(t)
# print("Collecting good leaves...got (%d)" % (len(gl)))


# dump(meshes)