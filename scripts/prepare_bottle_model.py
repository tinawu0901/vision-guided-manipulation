#!/usr/bin/env python3
"""Convert the attributed Fuel bottle to a lightweight, Z-up COLLADA asset."""

from pathlib import Path
import hashlib
import math
import urllib.request
import xml.etree.ElementTree as ET

import trimesh
from trimesh.visual.material import SimpleMaterial


SOURCE = 'https://fuel.gazebosim.org/1.0/iche033/models/Water%20Bottle/3/files/meshes/WaterBottle.glb'
root = Path(__file__).resolve().parents[1]
source = Path.home() / 'vision_guided_ws/work/models/water_bottle/WaterBottle.glb'
source.parent.mkdir(parents=True, exist_ok=True)
if not source.exists():
    urllib.request.urlretrieve(SOURCE, source)
scene = trimesh.load(source, force='scene')
mesh = list(scene.dump())[0]
mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2, [1, 0, 0]))
mesh.apply_scale(.22 / mesh.extents[2])
bounds = mesh.bounds
mesh.apply_translation([-bounds[:, 0].mean(), -bounds[:, 1].mean(), -bounds[0, 2]])
output = root / 'vision_guided_simulation/models/water_bottle'
output.mkdir(parents=True, exist_ok=True)
image = mesh.visual.material.baseColorTexture.convert('RGB')
image.thumbnail((512, 512))
image.save(output / 'bottle_texture.jpg', quality=90)
image.filename = 'bottle_texture.jpg'
mesh.visual.material = SimpleMaterial(image=image, diffuse=[255, 255, 255, 255])
content = trimesh.exchange.dae.export_collada(mesh)
document = ET.fromstring(content)
ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
document.find('c:asset/c:up_axis', ns).text = 'Z_UP'
tag = lambda name: '{' + ns['c'] + '}' + name
# The exporter retains UVs but omits the SimpleMaterial texture binding.
images = ET.SubElement(document, tag('library_images'))
texture = ET.SubElement(images, tag('image'), id='bottle-image')
ET.SubElement(texture, tag('init_from')).text = 'bottle_texture.jpg'
profile = document.find('.//c:profile_COMMON', ns)
surface_param = ET.Element(tag('newparam'), sid='bottle-surface')
surface = ET.SubElement(surface_param, tag('surface'), type='2D')
ET.SubElement(surface, tag('init_from')).text = 'bottle-image'
sampler_param = ET.Element(tag('newparam'), sid='bottle-sampler')
sampler = ET.SubElement(sampler_param, tag('sampler2D'))
ET.SubElement(sampler, tag('source')).text = 'bottle-surface'
profile.insert(0, surface_param)
profile.insert(1, sampler_param)
diffuse = profile.find('.//c:diffuse', ns)
diffuse.clear()
ET.SubElement(diffuse, tag('texture'), texture='bottle-sampler', texcoord='UVSET0')
ambient = profile.find('.//c:ambient/c:color', ns)
ambient.text = '0.5 0.5 0.5 1.0'
for binding in document.findall('.//c:instance_material', ns):
    ET.SubElement(binding, tag('bind_vertex_input'), semantic='UVSET0',
                  input_semantic='TEXCOORD', input_set='0')
for element in document.findall('.//c:library_images/c:image/c:init_from', ns):
    element.text = 'bottle_texture.jpg'
ET.register_namespace('', ns['c'])
ET.ElementTree(document).write(output / 'bottle.dae', encoding='utf-8', xml_declaration=True)
print('Source SHA256:', hashlib.sha256(source.read_bytes()).hexdigest())
print('Converted bounds (m):', mesh.bounds.tolist())
print('Wrote:', output)
