import pyvista as pv
import numpy as np
from pymeshfix import _meshfix
import pymeshfix as mf
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay


def area(x1, y1, x2, y2, x3, y3):
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)


# A function to check whether point P(x, y)
# lies inside the triangle formed by
# A(x1, y1), B(x2, y2) and C(x3, y3)
def isInside(triangle, x, y):
    x1 = triangle[0][0]
    y1 = triangle[0][1]
    x2 = triangle[1][0]
    y2 = triangle[1][1]
    x3 = triangle[2][0]
    y3 = triangle[2][1]

    A = area(x1, y1, x2, y2, x3, y3)
    A1 = area(x, y, x2, y2, x3, y3)
    A2 = area(x1, y1, x, y, x3, y3)
    A3 = area(x1, y1, x2, y2, x, y)

    if A == A1 + A2 + A3:
        return True
    else:
        return False


def get_face_points(face, points):
    p1 = points[face[1], :]
    p2 = points[face[2], :]
    p3 = points[face[3], :]
    return [p1, p2, p3]


# def remove_duplicates():
#     # projection.plot(show_edges=True, line_width=2)
#     triang = projection.triangulate().clean()
#     #
#     faces = triang.faces.reshape(-1, 4)
#     points = triang.points
#
#     triangle = get_face_points(faces[0], points)
#
#     faces_to_remove = []
#     i = 0
#     for face in faces:
#         f_points = get_face_points(face, points)
#
#         b = True
#         for p in f_points:
#             b = isInside(triangle, p[0], p[1])
#             if not b:
#                 print("Xaxa")
#         if b:
#             faces_to_remove.append(i)
#             print(f"Face: {face} is duplicate")
#
#         b = True
#         i = i + 1
#     print(len(faces_to_remove))
#     n = triang.remove_cells(faces_to_remove)
#     print(triang)
#     print(n)
#     # print(n)


# def project_points_to_plane(points, plane_origin, plane_normal):
#     """Project points to a plane."""
#     vec = points - plane_origin
#     dist = np.dot(vec, plane_normal)
#     return points - np.outer(dist, plane_normal)

def project_points_to_plane(mesh, origin=None, normal=(1, 0, 0), inplace=False):
    """Project points of this mesh to a plane"""
    if not isinstance(mesh, (pv.PolyData)):
        raise TypeError('Please use surface meshes only.')
    import vtk
    if origin is None:
        origin = mesh.center
    if not inplace:
        mesh = mesh.copy()
    # Make plane
    normal = normal / np.linalg.norm(normal) # MUST HAVE MAGNITUDE OF 1
    plane = vtk.vtkPlane()
    plane.SetOrigin(origin)
    plane.SetNormal(normal)

    # Perform projection in place on the copied mesh
    f = lambda p: plane.ProjectPoint(p, p)
    np.apply_along_axis(f, 1, mesh.points)
    if not inplace:
        return mesh
    return


def get_projection_area(mesh):
    # Define a plane
    origin = [0, 0, 0]
    normal = [0, 1, 0]

    min_z = mesh.bounds[4]
    np.linspace(-0.67, 0, 10)

    for i in range(3):
        print(i)
        step = 0.11

        clipped = vereniki.clip('z', value=(i+1)*step, origin=(0, 0, min_z))
        clipped = clipped.clip('z', value=i*step, origin=(0, 0, min_z), invert=False)

        projected_points = project_points_to_plane(clipped, origin, normal).points
        print(projected_points.shape)
        print(np.unique(projected_points, axis=0).shape)
        projected_points = np.unique(projected_points, axis=0)

        # Create a polydata object with projected points
        polydata = pv.PolyData(projected_points)

        # Mesh using delaunay_2d and pyvista
        surface = polydata.delaunay_2d(alpha=0.5)


        # plot it
        pl = pv.Plotter()
        pl.add_mesh(mesh, style='wireframe')
        pl.add_mesh(surface, show_edges=True, color='white', opacity=0.5, label='Tessellated mesh')
        pl.add_mesh(
            pv.PolyData(clipped.points),
            color='red',
            render_points_as_spheres=True,
            point_size=5,
            label='Points to project',
        )
        pl.add_legend()
        pl.show()
        print(surface.area)


if __name__ == '__main__':
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/vereniki/meshes/vereniki_scaled3.stl")
    vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.3, invert=True)
    vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.45, invert=False)
    #

    # vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/boat/meshes/boat3.stl")
    # vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.3, invert=True)
    # vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.45, invert=False)

    get_projection_area(vereniki_upper_part)

    # min_z = vereniki.bounds[4]
    # print(min_z)
    # origin = [0, 0, 0]
    # normal = [0, 1, 0]
    # projected_points = project_points_to_plane(vereniki.points, origin, normal)
    # print(projected_points.shape)
    # print(np.unique(projected_points, axis=0).shape)
    # projected_points = np.unique(projected_points, axis=0)
    #
    #
    # clipped = vereniki.clip('z', value=0.4, origin=(0, 0, min_z))
    # clipped = clipped.clip('z', value=0.1, origin=(0, 0, min_z), invert=False)
    #
    # clipped_p = clipped.points
    #
    # p = pv.Plotter()
    # # p.add_mesh(vereniki, style='wireframe', color='blue', label='Input')
    # # p.add_mesh(box, opacity=0.3)
    # p.add_mesh(clipped, label='Clipped')
    # p.add_mesh(
    #     pv.PolyData(clipped_p),
    #     color='red',
    #     render_points_as_spheres=True,
    #     point_size=5,
    #     label='Points to project',
    # )
    # p.add_legend()
    # p.show()









