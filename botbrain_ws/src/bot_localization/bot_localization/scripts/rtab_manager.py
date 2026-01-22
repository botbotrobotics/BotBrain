#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from typing import Optional
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

from lifecycle_msgs.msg import Transition

from std_srvs.srv import Trigger, Empty
from rtabmap_msgs.srv import LoadDatabase
from bot_localization_interfaces.srv import SetLocalization, SetMapping, LoadDB, SaveDatabase, ListDbFiles, DeleteDB, SetDefaultMap

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import yaml

# Database directory path will be constructed dynamically based on robot_model parameter


class RtabmapManagerLifecycle(LifecycleNode):
    """
    Lifecycle-managed node that wraps RTAB-Map control services.
    Uses a dynamic database directory path based on robot_model parameter: <robot_model>_pkg/maps
    The package location is resolved using ament_index, making it workspace-location-independent.
    All database operations work with filenames only; full paths are constructed internally.

    Services exposed by THIS node (only available when ACTIVE):
      - ~/set_localization      (bot_localization_interfaces/SetLocalization)
      - ~/set_mapping           (bot_localization_interfaces/SetMapping)     # takes filename, constructs full path
      - ~/save_database         (bot_localization_interfaces/SaveDatabase)   # stops mapping, saves maps
      - ~/load_database         (bot_localization_interfaces/LoadDB)         # takes filename, constructs full path
      - ~/list_db_files         (bot_localization_interfaces/ListDbFiles)    # lists .db files in dynamic directory
      - ~/delete_database       (bot_localization_interfaces/DeleteDB)       # deletes .db file by filename
      - ~/get_current_database  (std_srvs/Trigger)                           # returns currently loaded database name
      - ~/set_default_map       (bot_localization_interfaces/SetDefaultMap)  # sets default_map in robot_config.yaml

    Parameters:
      - robot_model           (string): Robot model name used to construct database path (required)
      - service_timeout_sec   (double): Timeout (sec) for waiting on downstream services (default: 15.0)
      
    Note: database_path fields in SetMapping and LoadDB services now expect filenames only,
    not full paths. The full path is constructed by joining the filename with the dynamic database directory.
    """

    def __init__(self):
        super().__init__('rtabmap_manager_lifecycle')

        # Parameters
        self.declare_parameter('robot_model', '')
        self.declare_parameter('mapping.database_path', '')
        self.declare_parameter('mapping.clear_on_load', False)
        self.declare_parameter('load.database_path', '')
        self.declare_parameter('load.clear_on_load', False)
        self.declare_parameter('service_timeout_sec', 60.0)

        # RTAB-Map service clients (created in on_configure)
        self.cli_set_loc = None
        self.cli_set_map = None
        self.cli_load_db = None

        # Our service servers (created in on_activate)
        self.srv_set_loc = None
        self.srv_set_map = None
        self.srv_save_db = None
        self.srv_load_db = None
        self.srv_list_db_files = None
        self.srv_delete_db = None
        self.srv_get_current_db = None
        self.srv_set_default_map = None

        self.callback_group = ReentrantCallbackGroup()

        # Track the currently loaded database name (filename only)
        self.current_database_name = "rtabmap.db"

        self.get_logger().info('Initialized (UNCONFIGURED).')

    # ---------------------------
    # Lifecycle transitions
    # ---------------------------

    def on_configure(self, state: State):
        self.get_logger().info('Configuring: creating RTAB-Map clients and ensuring availability...')

        timeout_sec = float(self.get_parameter('service_timeout_sec').value)


        # Create clients
        self.cli_set_loc = self.create_client(Empty, 'rtabmap/set_mode_localization', callback_group=self.callback_group)
        self.cli_set_map = self.create_client(Empty, 'rtabmap/set_mode_mapping', callback_group=self.callback_group)
        self.cli_load_db = self.create_client(LoadDatabase, 'rtabmap/load_database', callback_group=self.callback_group)

        if not self.cli_set_loc.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Service 'rtabmap/set_mode_localization' not available.")
            self.on_cleanup(state) 
            return TransitionCallbackReturn.FAILURE
        if not self.cli_set_map.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Service 'rtabmap/set_mode_mapping' not available.")
            self.on_cleanup(state) 
            return TransitionCallbackReturn.FAILURE
        if not self.cli_load_db.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Service 'rtabmap/load_database' not available.")
            self.on_cleanup(state) 
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("All services are available. Configuration successful.")
        return TransitionCallbackReturn.SUCCESS    
    

    def on_activate(self, state: State):
        self.get_logger().info('Activating: creating service servers...')

        # Create our public service servers
        self.srv_set_loc = self.create_service(SetLocalization, 'set_localization', self._srv_set_localization, callback_group=self.callback_group)
        self.srv_set_map = self.create_service(SetMapping, 'set_mapping', self._srv_set_mapping, callback_group=self.callback_group)
        self.srv_save_db = self.create_service(SaveDatabase, 'save_database', self._srv_save_database, callback_group=self.callback_group)
        self.srv_load_db = self.create_service(LoadDB, 'load_database', self._srv_load_database, callback_group=self.callback_group)
        self.srv_list_db_files = self.create_service(ListDbFiles, 'list_db_files', self._srv_list_db_files, callback_group=self.callback_group)
        self.srv_delete_db = self.create_service(DeleteDB, 'delete_database', self._srv_delete_database, callback_group=self.callback_group)
        self.srv_get_current_db = self.create_service(Trigger, 'get_current_database', self._srv_get_current_database, callback_group=self.callback_group)
        self.srv_set_default_map = self.create_service(SetDefaultMap, 'set_default_map', self._srv_set_default_map, callback_group=self.callback_group)

        self.get_logger().info('Services are UP.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating: destroying service servers...')
        if self.srv_set_loc:
            self.destroy_service(self.srv_set_loc)
        if self.srv_set_map:
            self.destroy_service(self.srv_set_map)
        if self.srv_save_db:
            self.destroy_service(self.srv_save_db)
        if self.srv_load_db:
            self.destroy_service(self.srv_load_db)
        if self.srv_list_db_files:
            self.destroy_service(self.srv_list_db_files)
        if self.srv_delete_db:
            self.destroy_service(self.srv_delete_db)
        if self.srv_get_current_db:
            self.destroy_service(self.srv_get_current_db)
        if self.srv_set_default_map:
            self.destroy_service(self.srv_set_default_map)

        self.get_logger().info('Service servers are DOWN.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up: destroying clients...')
        if self.cli_set_loc is not None:
            self.destroy_client(self.cli_set_loc)
        if self.cli_set_map is not None:
            self.destroy_client(self.cli_set_map)
        if self.cli_load_db is not None:
            self.destroy_client(self.cli_load_db)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info('Shutting down.')
        return self.on_cleanup(state)
    
    def _get_database_directory(self) -> str:
        """Get the database directory path based on robot_model parameter.
        Always uses the source directory for database storage.
        """
        robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        if not robot_model:
            raise ValueError("robot_model parameter is required but not set")

        package_name = f"{robot_model}_pkg"

        try:
            # Get the package share directory from ament_index (points to install)
            pkg_share = get_package_share_directory(package_name)

            # Navigate to source directory: workspace/install/<package>/share -> workspace/src/<package>
            src_maps_dir = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..', 'src', package_name, 'maps'))

            if not os.path.exists(src_maps_dir):
                raise ValueError(f"Source maps directory does not exist: {src_maps_dir}")

            if not os.path.isdir(src_maps_dir):
                raise ValueError(f"Source maps path is not a directory: {src_maps_dir}")

            self.get_logger().info(f"Using source maps directory: {src_maps_dir}")
            return src_maps_dir

        except PackageNotFoundError:
            raise ValueError(f"Package '{package_name}' not found. Make sure it is installed and sourced.")
    
    def _construct_full_path(self, filename: str) -> str:
        """Construct full database path from filename using the dynamic database directory."""
        database_dir = self._get_database_directory()
        return os.path.join(database_dir, filename)
    
    def _validate_database_path(self, db_path: str, allow_create: bool = False) -> tuple[bool, str]:
        """Validate database path exists and has proper extension.
        
        Args:
            db_path: Path to database file
            allow_create: If True, allows non-existing paths (for creating new databases)
        """
        if not db_path:
            return False, "Database path cannot be empty"
        
        if not os.path.exists(db_path):
            if allow_create:
                # Check if the parent directory exists for creating new database
                parent_dir = os.path.dirname(db_path)
                if parent_dir and not os.path.exists(parent_dir):
                    return False, f"Parent directory does not exist: {parent_dir}"
                self.get_logger().info(f"Database will be created at: {db_path}")
                return True, "Database will be created at specified path"
            else:
                return False, f"Database file does not exist: {db_path}"
        
        if not os.path.isfile(db_path):
            return False, f"Database path is not a file: {db_path}"
        
        if not db_path.lower().endswith('.db'):
            self.get_logger().warn(f"Database file does not have .db extension: {db_path}")
        
        return True, "Valid database path"

    def _srv_set_localization(self, request: SetLocalization.Request, response: SetLocalization.Response) -> SetLocalization.Response:
        self.get_logger().info("Request received: entering localization mode...")
        future = self.set_localization_mode()
        
        if future.done():
            try:
                future.result()
                response.success = True
                response.message = "Successfully switched RTAB-Map to localization mode."
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f"Failed to call rtabmap service: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Call to 'rtabmap/set_mode_localization' timed out."
            self.get_logger().error(response.message)
            
        return response
    
    def _srv_set_mapping(self, request: SetMapping.Request, response: SetMapping.Response) -> SetMapping.Response:
        self.get_logger().info("Request received for 'set_mapping' service.")
        if request.database_path:
            # Construct full path from filename
            full_path = self._construct_full_path(request.database_path)
            
            # Validate database path first (allow creation for mapping)
            is_valid, validation_msg = self._validate_database_path(full_path, allow_create=True)
            if not is_valid:
                response.success = False
                response.message = f"Invalid database filename '{request.database_path}': {validation_msg}"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info(f"Loading database from: {full_path} (filename: {request.database_path})")
            load_future = self.load_database(full_path, request.clear_db)

            if load_future.done():
                try:
                    load_future.result()  # Check for exceptions
                    self.get_logger().info("Database loaded successfully.")
                    # Update the current database name tracker
                    self.current_database_name = request.database_path
                except Exception as e:
                    response.success = False
                    response.message = f"Failed to load database: {e}"
                    self.get_logger().error(response.message)
                    return response
            else:
                response.success = False
                response.message = "Call to 'rtabmap/load_database' timed out."
                self.get_logger().error(response.message)
                return response

        self.get_logger().info("Switching to mapping mode...")
        map_future = self.set_mapping_mode()

        if map_future.done():
            try:
                map_future.result()  # Check for exceptions
                response.success = True
                response.message = "Successfully set mapping mode."
                if request.database_path:
                    response.message = f"Successfully loaded database '{request.database_path}' and set mapping mode."
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f"Failed to set mapping mode: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Call to 'rtabmap/set_mode_mapping' timed out."
            self.get_logger().error(response.message)
            
        return response

    def _srv_load_database(self, request: LoadDB.Request, response: LoadDB.Response) -> LoadDB.Response:
        """
        Callback for the '~/load_database' service. Calls the downstream rtabmap service
        with the filename provided in the request (constructs full path internally).
        """
        self.get_logger().info("Request received for 'load_database' service.")

        # Construct full path from filename
        full_path = self._construct_full_path(request.database_path)

        # Validate database path (must exist for loading)
        is_valid, validation_msg = self._validate_database_path(full_path, allow_create=False)
        if not is_valid:
            response.success = False
            response.message = f"Invalid database filename '{request.database_path}': {validation_msg}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f"Calling downstream load_database service for file: '{request.database_path}' (full path: {full_path})")
        future = self.load_database(full_path, request.clear_db)

        if future.done():
            try:
                future.result()
                response.success = True
                response.message = f"Successfully loaded database '{request.database_path}'."
                self.get_logger().info(response.message)
                # Update the current database name tracker
                self.current_database_name = request.database_path
            except Exception as e:
                response.success = False
                response.message = f"Failed to load database: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Call to 'rtabmap/load_database' timed out."
            self.get_logger().error(response.message)

        return response

    def _srv_save_database(self, request: SaveDatabase.Request, response: SaveDatabase.Response) -> SaveDatabase.Response:
        """
        Callback for the 'save_database' service. Switches RTAB-Map to localization mode
        and logs that the map was saved. Future enhancement will upload to cloud.
        """
        self.get_logger().info("Request received for 'save_database' service.")

        # Get the database path to display where it's being saved
        try:
            database_dir = self._get_database_directory()
            # Assuming the database filename from parameters or default
            db_filename = self.get_parameter('mapping.database_path').get_parameter_value().string_value
            if not db_filename:
                db_filename = "rtabmap.db"  # Default filename
            db_full_path = os.path.join(database_dir, db_filename)
        except Exception as e:
            self.get_logger().warn(f"Could not determine database path: {e}")
            db_full_path = "unknown"

        # Switch to localization mode (this stops mapping and saves the current session)
        self.get_logger().info("Switching to localization mode to save current mapping session...")
        future = self.set_localization_mode()

        if future.done():
            try:
                future.result()  # Check for exceptions
                response.success = True
                response.message = f"Maps was saved successfully to: {db_full_path}. Switched to localization mode."
                self.get_logger().info(f"✓ Maps was saved successfully to: {db_full_path}")
                self.get_logger().info("Note: Future enhancement will upload database to cloud.")
            except Exception as e:
                response.success = False
                response.message = f"Failed to save maps: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Failed to save maps: timeout switching to localization mode."
            self.get_logger().error(response.message)

        return response

    def _srv_list_db_files(self, request: ListDbFiles.Request, response: ListDbFiles.Response) -> ListDbFiles.Response:
        """
        Callback for the '~/list_db_files' service. Lists all .db files in the dynamic database directory.
        """
        try:
            database_dir = self._get_database_directory()
            self.get_logger().info(f"Request received for 'list_db_files' service, searching in: '{database_dir}'")
            
            # Validate that the database directory exists
            if not os.path.exists(database_dir):
                response.success = False
                response.message = f"Database directory does not exist: {database_dir}"
                self.get_logger().error(response.message)
                return response
            
            if not os.path.isdir(database_dir):
                response.success = False
                response.message = f"Database path is not a directory: {database_dir}"
                self.get_logger().error(response.message)
                return response
            
            # List all .db files in the directory
            db_files = []
            try:
                for filename in os.listdir(database_dir):
                    if filename.lower().endswith('.db'):
                        db_files.append(filename)
                
                db_files.sort()  # Sort alphabetically for consistent output
                
                response.success = True
                response.db_files = db_files
                response.message = f"Found {len(db_files)} .db file(s) in database directory"
                self.get_logger().info(f"Found {len(db_files)} .db file(s): {db_files}")
                
            except PermissionError:
                response.success = False
                response.message = f"Permission denied accessing database directory: {database_dir}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def _srv_delete_database(self, request: DeleteDB.Request, response: DeleteDB.Response) -> DeleteDB.Response:
        """
        Callback for the '~/delete_database' service. Deletes a .db file from the dynamic database directory.
        """
        try:
            database_dir = self._get_database_directory()
            self.get_logger().info(f"Request received for 'delete_database' service, filename: '{request.database_path}'")
            
            # Construct full path from filename
            full_path = self._construct_full_path(request.database_path)
            
            # Validate that the database directory exists
            if not os.path.exists(database_dir):
                response.success = False
                response.message = f"Database directory does not exist: {database_dir}"
                self.get_logger().error(response.message)
                return response
            
            if not os.path.isdir(database_dir):
                response.success = False
                response.message = f"Database path is not a directory: {database_dir}"
                self.get_logger().error(response.message)
                return response
            
            # Validate the database file exists
            if not os.path.exists(full_path):
                response.success = False
                response.message = f"Database file does not exist: {request.database_path}"
                self.get_logger().error(response.message)
                return response
            
            if not os.path.isfile(full_path):
                response.success = False
                response.message = f"Database path is not a file: {request.database_path}"
                self.get_logger().error(response.message)
                return response
            
            # Safety check: ensure the file has .db extension
            if not request.database_path.lower().endswith('.db'):
                response.success = False
                response.message = f"Only .db files can be deleted, got: {request.database_path}"
                self.get_logger().error(response.message)
                return response
            
            # Delete the file
            try:
                os.remove(full_path)
                response.success = True
                response.message = f"Successfully deleted database file: {request.database_path}"
                self.get_logger().info(response.message)
                
            except PermissionError:
                response.success = False
                response.message = f"Permission denied deleting database file: {request.database_path}"
                self.get_logger().error(response.message)
            except OSError as e:
                response.success = False
                response.message = f"OS error deleting database file '{request.database_path}': {str(e)}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def _srv_get_current_database(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Callback for the '~/get_current_database' service. Returns the currently loaded database name.
        """
        self.get_logger().info("Request received for 'get_current_database' service.")

        response.success = True
        response.message = self.current_database_name
        self.get_logger().info(f"Current database: {self.current_database_name}")

        return response

    def _get_config_file_path(self) -> str:
        """Get the path to robot_config.yaml."""
        pkg_share = get_package_share_directory(f"{self.get_parameter('robot_model').get_parameter_value().string_value}_pkg")
        workspace_dir = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
        return os.path.join(workspace_dir, 'robot_config.yaml')

    def _srv_set_default_map(self, request: SetDefaultMap.Request, response: SetDefaultMap.Response) -> SetDefaultMap.Response:
        """
        Callback for the '~/set_default_map' service. Updates the default_map field in robot_config.yaml.
        """
        self.get_logger().info(f"Request received for 'set_default_map' service: '{request.map_name}'")

        map_name = request.map_name

        # Validate the map name
        if not map_name:
            response.success = False
            response.message = "Map name cannot be empty"
            self.get_logger().error(response.message)
            return response

        if not map_name.lower().endswith('.db'):
            response.success = False
            response.message = f"Map name must have .db extension, got: {map_name}"
            self.get_logger().error(response.message)
            return response

        # Check if the database file exists
        full_path = self._construct_full_path(map_name)
        if not os.path.exists(full_path):
            response.success = False
            response.message = f"Database file does not exist: {map_name}"
            self.get_logger().error(response.message)
            return response

        try:
            config_file = self._get_config_file_path()

            # Read the current config
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Update the default_map field
            config['robot_configuration']['default_map'] = map_name

            # Write the updated config
            with open(config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)

            response.success = True
            response.message = f"Successfully set default map to: {map_name}"
            self.get_logger().info(response.message)

        except FileNotFoundError:
            response.success = False
            response.message = f"Config file not found: {config_file}"
            self.get_logger().error(response.message)
        except PermissionError:
            response.success = False
            response.message = f"Permission denied writing to config file: {config_file}"
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to update config file: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def set_localization_mode(self):
        """Calls the downstream rtabmap service to enable localization mode."""
        rtabmap_request = Empty.Request()
        future = self.cli_set_loc.call_async(rtabmap_request)
        timeout_sec = self.get_parameter('service_timeout_sec').get_parameter_value().double_value
        
        # Wait for completion without blocking the executor
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.01)  # Small sleep to prevent busy waiting
            
        return future
    
    def set_mapping_mode(self):
        """Calls the downstream rtabmap service to enable mapping mode."""
        rtabmap_request = Empty.Request()
        future = self.cli_set_map.call_async(rtabmap_request)
        timeout_sec = self.get_parameter('service_timeout_sec').get_parameter_value().double_value
        
        # Wait for completion without blocking the executor
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.01)  # Small sleep to prevent busy waiting
            
        return future

    def load_database(self, db_path: str, clear: bool):
        """Calls the downstream rtabmap service to load a database."""
        rtabmap_request = LoadDatabase.Request()
        rtabmap_request.database_path = db_path
        rtabmap_request.clear = clear
        future = self.cli_load_db.call_async(rtabmap_request)
        timeout_sec = self.get_parameter('service_timeout_sec').get_parameter_value().double_value
        
        # Wait for completion without blocking the executor
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.01)  # Small sleep to prevent busy waiting
            
        return future
    

def main(args=None):
    rclpy.init(args=args)
    
    node = RtabmapManagerLifecycle()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    