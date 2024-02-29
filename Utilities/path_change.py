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
            folder_elem = root.find('folder')
            image_name = root.find('filename').text

            if path_elem is not None:
                folder_elem.text = "images"
                path_elem.text = new_path + image_name
            tree.write(file_path)


def main():
    parser = argparse.ArgumentParser(description='Change path in XML files.')
    parser.add_argument('directory', type=str, help='Directory of XML files')
    parser.add_argument('new_path', type=str, help='New path to replace in XML files')
    args = parser.parse_args()
    
    change_xml_path(args.directory, args.new_path)


# Usage:
# python path_change.py /path/to/xmls /new/path/to/replace
if __name__ == '__main__':
    main()