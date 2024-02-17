import os
import xml.etree.ElementTree as ET
import argparse

def change_xml_path(directory, new_path):
    for filename in os.listdir(directory):
        if filename.endswith('.xml'):
            file_path = os.path.join(directory, filename)
                 
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            path_elem = root.find('path')
            
            if path_elem is not None:
                path_elem.text = new_path
            
            tree.write(file_path)


def main():
    parser = argparse.ArgumentParser(description='Change path in XML files.')
    parser.add_argument('directory', type=str, help='Directory of XML files')
    parser.add_argument('new_path', type=str, help='New path to replace in XML files')
    args = parser.parse_args()
    
    change_xml_path(args.directory, args.new_path)
    
if __name__ == '__main__':
    main()