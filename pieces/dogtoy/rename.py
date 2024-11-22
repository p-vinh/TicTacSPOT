import os

def prepend_to_filenames(directory, prefix):
    # Ensure the directory exists
    if not os.path.isdir(directory):
        print(f"The directory {directory} does not exist.")
        return

    # List all files in the directory
    for filename in os.listdir(directory):
        # Create the full file path
        old_path = os.path.join(directory, filename)
        
        # Skip directories and hidden files
        if os.path.isfile(old_path) and not filename.startswith('.'):
            # Construct the new file name
            new_filename = prefix + filename
            new_path = os.path.join(directory, new_filename)
            
            # Rename the file
            os.rename(old_path, new_path)
            print(f"Renamed {old_path} to {new_path}")

# Specify the folder and prefix
folder_path = 'C:\Users\CPP-UAV-CYBER-A\Spot.Data\TicTacSPOT\pieces\dogtoy\images_and_labels'
file_prefix = 'dark_'

# Run the function
prepend_to_filenames(folder_path, file_prefix)
