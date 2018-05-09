# -*- coding: utf-8 -*-
"""

@author: zhangc
"""
import argparse
from xml.etree.ElementTree import ElementTree,Element
import random 

parser = argparse.ArgumentParser()
parser.add_argument("data",type=str,help="pusher, striker, thrower")
args = parser.parse_args()

def read_xml(in_path):
  '''Read and parse the XML file.
    in_path: xml path
    return: ElementTree'''
  tree = ElementTree()
  tree.parse(in_path)
  return tree
 
def write_xml(tree, out_path):
  '''Write the XML file.
    tree: xml tree
    out_path: Write-out path'''
  tree.write(out_path, encoding="utf-8",xml_declaration=True)
 
def if_match(node, kv_map):
  '''Determine whether a node contains all incoming parameter properties.
    node: node
    kv_map: Map of attributes and attribute values'''
  for key in kv_map:
    if node.get(key) != kv_map.get(key):
      return False
  return True
 
#---------------search -----
def find_nodes(tree, path):
  '''Find all the nodes that match a path.
    tree: xml tree
    path: Node path'''
  return tree.findall(path)
 
def get_node_by_keyvalue(nodelist, kv_map):
  '''Locate the corresponding node according to the attribute and attribute value, 
     and return the node.
    nodelist: Node list
    kv_map: Matching property and attribute value map'''
  result_nodes = []
  for node in nodelist:
    if if_match(node, kv_map):
      result_nodes.append(node)
  return result_nodes
 
#---------------change -----
def change_node_properties(nodelist, kv_map, is_delete=False):
  '''Modify / add / delete node's property and attribute values.
    nodelist: Node list
    kv_map:Attribute and attribute value map'''
  for node in nodelist:
    for key in kv_map:
      if is_delete:
        if key in node.attrib:
          del node.attrib[key]
      else:
        node.set(key, kv_map.get(key))
 
def change_node_text(nodelist, text, is_add=False, is_delete=False):
  '''Changing / adding / deleting the text of a node
    nodelist:Node list
    text : Updated text'''
  for node in nodelist:
    if is_add:
      node.text += text
    elif is_delete:
      node.text = ""
    else:
      node.text = text

 
 
if __name__ == "__main__":
  if args.data=="pusher":
    for i in range(140):
        tree = read_xml("/home/zhangc/gym/gym/envs/mujoco/assets/pusher.xml") 
        nodes_geom_0 = find_nodes(tree, "worldbody/body/body/body/body/geom")
        nodes_body_0 = find_nodes(tree, "worldbody/body/body/body/body/body")
        nodes_geom_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/geom")
        nodes_body_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/body")      
        random_0 = random.uniform(0.1, 0.5)
        random_1 = random.uniform(0.2, 0.6)
        result_nodes_geom0 = get_node_by_keyvalue(nodes_geom_0, {"name":"ua"})
        change_node_properties(result_nodes_geom0, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_1,0,0)})
        
        result_nodes_body0 = get_node_by_keyvalue(nodes_body_0, {"name":"r_elbow_flex_link"})
        change_node_properties(result_nodes_body0, {"pos": "%f %f %f" %(random_1,0,0)})
      
        result_nodes_geom1 = get_node_by_keyvalue(nodes_geom_1, {"name":"fa"})
        change_node_properties(result_nodes_geom1, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_0,0,0)})
   
        result_nodes_body1 = get_node_by_keyvalue(nodes_body_1, {"name":"r_wrist_flex_link"})
        change_node_properties(result_nodes_body1, {"pos": "%f %f %f" %(random_0+0.03,0,0)})
        write_xml(tree, "/home/zhangc/gym/gym/envs/mujoco/assets/pusher%d.xml"% i)

  elif args.data=="striker":
    for i in range(140):
        tree = read_xml("/home/zhangc/gym/gym/envs/mujoco/assets/striker.xml") 
        nodes_geom_0 = find_nodes(tree, "worldbody/body/body/body/body/geom")
        nodes_body_0 = find_nodes(tree, "worldbody/body/body/body/body/body")
        nodes_geom_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/geom")
        nodes_body_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/body")      
        random_0 = random.uniform(0.1, 0.5)
        random_1 = random.uniform(0.2, 0.6)
        result_nodes_geom0 = get_node_by_keyvalue(nodes_geom_0, {"name":"ua"})
        change_node_properties(result_nodes_geom0, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_1,0,0)})
        
        result_nodes_body0 = get_node_by_keyvalue(nodes_body_0, {"name":"r_elbow_flex_link"})
        change_node_properties(result_nodes_body0, {"pos": "%f %f %f" %(random_1,0,0)})
      
        result_nodes_geom1 = get_node_by_keyvalue(nodes_geom_1, {"name":"fa"})
        change_node_properties(result_nodes_geom1, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_0,0,0)})
   
        result_nodes_body1 = get_node_by_keyvalue(nodes_body_1, {"name":"r_wrist_flex_link"})
        change_node_properties(result_nodes_body1, {"pos": "%f %f %f" %(random_0+0.03,0,0)})
        write_xml(tree, "/home/zhangc/gym/gym/envs/mujoco/assets/striker%d.xml"% i)

  elif args.data=="thrower":
    for i in range(140):
        tree = read_xml("/home/zhangc/gym/gym/envs/mujoco/assets/thrower.xml") 
        nodes_geom_0 = find_nodes(tree, "worldbody/body/body/body/body/geom")
        nodes_body_0 = find_nodes(tree, "worldbody/body/body/body/body/body")
        nodes_geom_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/geom")
        nodes_body_1 = find_nodes(tree, "worldbody/body/body/body/body/body/body/body/body")      
        random_0 = random.uniform(0.1, 0.5)
        random_1 = random.uniform(0.2, 0.6)
        result_nodes_geom0 = get_node_by_keyvalue(nodes_geom_0, {"name":"ua"})
        change_node_properties(result_nodes_geom0, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_1,0,0)})
        
        result_nodes_body0 = get_node_by_keyvalue(nodes_body_0, {"name":"r_elbow_flex_link"})
        change_node_properties(result_nodes_body0, {"pos": "%f %f %f" %(random_1,0,0)})
      
        result_nodes_geom1 = get_node_by_keyvalue(nodes_geom_1, {"name":"fa"})
        change_node_properties(result_nodes_geom1, {"fromto": "%f %f %f %f %f %f" %(0,0,0,random_0,0,0)})
   
        result_nodes_body1 = get_node_by_keyvalue(nodes_body_1, {"name":"r_wrist_flex_link"})
        change_node_properties(result_nodes_body1, {"pos": "%f %f %f" %(random_0+0.03,0,0)})
        write_xml(tree, "/home/zhangc/gym/gym/envs/mujoco/assets/thrower%d.xml"% i)
        
  else:
    print("Invalid argument")
