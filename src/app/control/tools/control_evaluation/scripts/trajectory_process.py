# -----------------------------------------------------------------------------
# @file           trajectory_process.py 
# @brief          control evaluation tool by using waypoint planner and lanelet maps
#
# @authors        Seounghoon Park (sunghoon8585@gmail.com)
#
# @date           2024-12-30      Created by Seounghoon Park
# -----------------------------------------------------------------------------

from lxml import etree  # Add this for parsing the OSM file
import os
from lanelet2.io import Origin, load
from pyproj import Proj, Transformer 

class TrajectoryProcess(object):
    def __init__(self, map_directory, ref_lat, ref_lon):
        self.map_directory = map_directory
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.lanelet_maps = []  
        self.osm_file_paths = []
        
        self.trajectories = []

        # Create a pyproj Transformer object for local cartesian conversion
        # EPSG:4326 is WGS84 (lat/lon), and we convert it to a local cartesian system
        self.transformer = Transformer.from_crs("EPSG:4326", f"+proj=tmerc +lat_0={ref_lat} +lon_0={ref_lon} +datum=WGS84")



    def convert_to_local(self, lat, lon):
        """
        Convert WGS84 (latitude, longitude) to local cartesian coordinates
        """
        try:
            x, y = self.transformer.transform(lat, lon)
            return x, y
        except Exception as e:
            print(f"Error in coordinate conversion: {e}")
            return 0.0, 0.0



    def load_osm_file(self, file_path):
        """
        Parse OSM file and extract nodes and centerlines
        """
        trajectories = []
        try:
            tree = etree.parse(file_path)
            root = tree.getroot()

            # Extract all nodes with their lat/lon
            nodes = {}
            for node in root.findall("node"):
                node_id = node.get("id")
                lat = node.get("lat")
                lon = node.get("lon")
                if node_id and lat and lon:
                    nodes[node_id] = (float(lat), float(lon))

            # Extract relations with centerline
            for relation in root.findall("relation"):
                centerline_points = []
                for member in relation.findall("member"):
                    if member.get("role") == "centerline" and member.get("type") == "way":
                        way_id = member.get("ref")

                        # Find corresponding way
                        for way in root.findall("way"):
                            if way.get("id") == way_id:
                                for nd in way.findall("nd"):
                                    ref = nd.get("ref")
                                    if ref in nodes:
                                        lat, lon = nodes[ref]
                                        x, y = self.convert_to_local(lat, lon)
                                        centerline_points.append((x, y))
                                break  # Stop searching once the way is found
                if centerline_points:
                    trajectories.append(centerline_points)

        except Exception as e:
            print(f"Error loading OSM file {file_path}: {e}")
        
        return trajectories



    def load_lanelet_maps(self):
        """
        Load all OSM files in the map directory and process their centerlines
        """
        if not self.map_directory or not os.path.isdir(self.map_directory):
            print(f"Invalid map directory provided: {self.map_directory}")
            return

        try:
            for file_name in os.listdir(self.map_directory):
                if file_name.endswith(".osm"):
                    file_path = os.path.join(self.map_directory, file_name)
                    self.osm_file_paths.append(file_path)
                    print(f"Processing OSM file: {file_name}")
                    trajectories = self.load_osm_file(file_path)
                    self.trajectories.extend(trajectories)

            print(f"Processed {len(self.trajectories)} centerlines.")
        except Exception as e:
            print(f"Failed to process OSM files: {e}")




    def get_scenes(self):
        """
        Load OSM files and convert centerline points to local cartesian coordinates
        """
        self.load_lanelet_maps()
        return self.trajectories, self.osm_file_paths