import os
import zipfile

def unpack_zip_files(directory):
    """
    Unpacks all ZIP files in the given directory.

    Args:
        directory (str): The path to the directory containing ZIP files.
    """
    if not os.path.exists(directory):
        print(f"Directory '{directory}' does not exist.")
        return

    # Iterate through all files in the directory
    for file_name in os.listdir(directory):
        file_path = os.path.join(directory, file_name)

        # Check if the file is a ZIP file
        if zipfile.is_zipfile(file_path):
            try:
                # Create a folder for the extracted files
                output_dir = os.path.join(directory, os.path.splitext(file_name)[0])
                os.makedirs(output_dir, exist_ok=True)

                # Extract the ZIP file
                with zipfile.ZipFile(file_path, 'r') as zip_ref:
                    zip_ref.extractall(output_dir)
                print(f"Unpacked '{file_name}' to '{output_dir}'")
            except Exception as e:
                print(f"Error unpacking '{file_name}': {e}")
        else:
            print(f"'{file_name}' is not a valid ZIP file.")

if __name__ == "__main__":

    # Specify the directory containing the ZIP files
    # directory = '/home/XXXXXXX/subt_models/zip_files'
    directory = '/home/XXXXXXX/Downloads/zips'
    unpack_zip_files(directory)