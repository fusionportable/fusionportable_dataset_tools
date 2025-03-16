import argparse
import os
import subprocess
from typing import List
import gdown

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Download FusionPortable dataset')
    parser.add_argument('--version', type=str, required=True, choices=['fpv1', 'fpv2'],
                        help='Dataset version to download')
    parser.add_argument('--dataset_dir', type=str, default='data_FusionPortable',
                        help='Root directory for dataset storage')
    parser.add_argument('--data_types', nargs='+', required=True,
                        choices=['sensor_data', 'groundtruth', 'calibration_files', 'all'],
                        help='Dataset components to download')
    parser.add_argument('--file_names', nargs='+', required=False,
                        help='The specific bag_name will be downloaded (e.g., vehicle_campus00) or all')
    return parser.parse_args()

def download_file(file_id: str, filename: str, dataset_dir: str, component: str):
    """Download a dataset filename using gdown with proper permission handling."""
    component_dir = os.path.join(dataset_dir, component)
    os.makedirs(component_dir, exist_ok=True)
    output_path = os.path.join(component_dir, filename)
    print(f"⏳ Downloading {filename} to {output_path} ...")
    
    try:
        # First try direct download
        gdown.download(f"https://drive.google.com/uc?id={file_id}", output_path, quiet=False)
    except Exception as e:
        print(f"⚠️ Direct download failed: {str(e)}. Trying with cookies...")
        
        # Attempt with cookies for permission issues
        cookie_path = os.path.expanduser("~/cookies.txt")
        if not os.path.exists(cookie_path):
            raise RuntimeError("Cookie file not found. Export cookies from your browser!")
            
        try:
            gdown.download(
                f"https://drive.google.com/uc?id={file_id}",
                output_path,
                quiet=False,
                use_cookies=True,
                verify=False,  # Only if you trust the source
                cookies=cookie_path
            )
        except Exception as cookie_error:
            print(f"❌ Cookie-based download failed: {str(cookie_error)}")
            print("Manual steps required:")
            print(f"1. Visit https://drive.google.com/file/d/{file_id}/view")
            print("2. Confirm download through browser")
            print("3. Place downloaded file in:", output_path)
            raise

def decompress_and_clean(filename: str, dataset_dir: str, component: str):
    """Decompress an archive and remove it."""
    component_dir = os.path.join(dataset_dir, component)
    filepath = os.path.join(component_dir, filename)
    
    print(f"🔧 Decompressing {filename}...")
    try:
        subprocess.run(["7z", "x", filepath, f"-o{component_dir}"], check=True)
    except subprocess.CalledProcessError:
        print(f"❌ Failed to decompress {filename}")
        raise
    
    os.remove(filepath)
    print(f"✅ Removed archive {filename}")

def process_dataset(version: str, data_types: List[str], dataset_dir: str, file_names: List[str]):
    """Main processing function for dataset download and extraction."""

    if version == 'fpv1':
        raise ValueError("fpv1 is not supported.")
    elif version == 'fpv2':
        from fpv2_google_drive_link import FILE_MAPPING
    else:
        raise ValueError(f"{version} is not supported.")
    
    # Handle 'all' data types
    if 'all' in data_types:
        data_types = ['sensor_data', 'groundtruth', 'calibration_files']
    
    for component in data_types:
        if component not in FILE_MAPPING:
            print(f"Component {component} not available in version {version}")
            continue

        print(f'Download {component}')
        
        if component == 'sensor_data':
            if 'all' in file_names:
                file_names = [name in FILE_MAPPING[component].keys()]
            
            for filename in file_names:
                try:
                    file_id = FILE_MAPPING[component][filename]
                    download_file(file_id, filename, dataset_dir, component)
                    decompress_and_clean(filename, dataset_dir, component)
                    print(f"✨ {component}/{filename} processed successfully")
                except Exception as e:
                    print(f"❌ Error processing {component}/{filename}: {str(e)}")
        else:
            for filename, file_id in FILE_MAPPING[component].items():
                try:
                    download_file(file_id, filename, dataset_dir, component)
                    decompress_and_clean(filename, dataset_dir, component)
                    print(f"✨ {component}/{filename} processed successfully")
                except Exception as e:
                    print(f"❌ Error processing {component}/{filename}: {str(e)}")

if __name__ == "__main__":
    args = parse_arguments()
    
    # Validate 7z installation
    try:
        subprocess.run(["7z"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ 7z utility not found. Please install 7-zip:")
        print("   Ubuntu/Debian: sudo apt install p7zip-full")
        print("   macOS: brew install p7zip")
        exit(1)

    process_dataset(args.version, args.data_types, args.dataset_dir, args.file_names)
    print("🏁 All operations completed!")
