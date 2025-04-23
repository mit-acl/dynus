import os

def rename_files_in_directory(directory):
    """
    Renames all files in the given directory that start and end with single quotes.
    The renamed files will be converted to lowercase and replace spaces with underscores.
    
    Args:
        directory (str): The path to the directory containing the files.
    """
    if not os.path.exists(directory):
        print(f"Directory '{directory}' does not exist.")
        return

    for file_name in os.listdir(directory):
        # Check if the file name starts and ends with single quotes
        # Remove the quotes and replace spaces with underscores
        new_name = file_name.replace(" ", "_").lower()
        old_path = os.path.join(directory, file_name)
        new_path = os.path.join(directory, new_name)

        try:
            os.rename(old_path, new_path)
            print(f"Renamed: '{file_name}' -> {new_name}")
        except Exception as e:
            print(f"Error renaming '{file_name}': {e}")

if __name__ == "__main__":
    # Specify the directory containing the files
    directory = "/home/XXXXXXX/.gazebo/models"
    rename_files_in_directory(directory)
