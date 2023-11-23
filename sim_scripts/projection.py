import pyvista as pv
import numpy as np
import vtk


def project_points_to_plane(mesh, origin=None, normal=(1, 0, 0), inplace=False):
    """Project points of this mesh to a plane"""
    if not isinstance(mesh, (pv.PolyData)):
        raise TypeError('Please use surface meshes only.')

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


def get_projection_area(mesh, normal, iterations=10, plot=False):
    min_z = mesh.bounds[4]
    max_z = mesh.bounds[5]
    step = (max_z - min_z)/iterations
    total_area = 0

    for i in range(iterations):
        # Take mesh slice
        clipped = vereniki.clip('z', value=(i+1)*step, origin=(0, 0, min_z))
        clipped = clipped.clip('z', value=i*step, origin=(0, 0, min_z), invert=False)

        # Project slice to plane and remove duplicate points
        projected_points = project_points_to_plane(clipped, normal=normal).points
        projected_points = np.unique(projected_points, axis=0)

        # Generate new surface from projected points and add its area
        polydata = pv.PolyData(projected_points)
        surface = polydata.delaunay_2d(alpha=0.3)
        total_area = total_area + surface.area

        if plot:
            # plot it
            pl = pv.Plotter()
            pl.add_mesh(mesh, style='wireframe')
            pl.add_mesh(surface, show_edges=True, color='white', opacity=0.5, label='New surface')
            pl.add_mesh(
                pv.PolyData(clipped.points),
                color='red',
                render_points_as_spheres=True,
                point_size=5,
                label='Points to project',
            )
            pl.add_legend()
            pl.show()
    return total_area


if __name__ == '__main__':
    vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/vereniki/meshes/vereniki_scaled3.stl")
    vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.3, invert=True)
    vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.45, invert=False)
    #

    # vereniki = pv.read("../../../../gz_ws/src/usv_simulation/models/boat/meshes/boat3.stl")
    # vereniki_lower_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.3, invert=True)
    # vereniki_upper_part = vereniki.clip('z', value=-((0.65 + 0.3) / 2) + 0.45, invert=False)

    area = get_projection_area(vereniki_lower_part, [1, 0, 0])
    print(area)










