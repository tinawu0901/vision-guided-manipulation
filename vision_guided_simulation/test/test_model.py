from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def test_bottle_coordinates_match_scene_units():
    model = Path(__file__).parents[1] / 'models/water_bottle/bottle.dae'
    root = ET.parse(model).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    assert root.find('c:asset/c:up_axis', ns).text == 'Z_UP'
    vertices = root.find('.//c:source[@id="verts-array"]/c:float_array', ns)
    points = np.fromstring(vertices.text, sep=' ').reshape(-1, 3)
    assert abs(points[:, 2].min()) < 1e-6
    assert abs(points[:, 2].max() - .22) < 1e-6
    assert abs(points[:, :2]).max() < .04601


def test_bottle_texture_has_complete_material_binding():
    model = Path(__file__).parents[1] / 'models/water_bottle/bottle.dae'
    root = ET.parse(model).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    image = root.find('c:library_images/c:image', ns)
    assert (model.parent / image.find('c:init_from', ns).text).is_file()
    texture = root.find('.//c:diffuse/c:texture', ns)
    binding = root.find('.//c:instance_material/c:bind_vertex_input', ns)
    assert texture.attrib['texcoord'] == binding.attrib['semantic']
    assert binding.attrib['input_semantic'] == 'TEXCOORD'
