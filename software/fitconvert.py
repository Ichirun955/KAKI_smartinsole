import os
from pathlib import Path
from fitparse import FitFile
import pandas as pd

def parse_fit_to_csv(fit_path: str | Path, output_csv_path: str | Path = None) -> pd.DataFrame:
    """
    Parses a single .FIT file (including custom Connect IQ developer fields)
    and exports it to a unique CSV file.
    """
    fit_path = Path(fit_path)
    
    if not fit_path.exists():
        print(f"[ERROR] File not found: {fit_path}")
        return None
    
    # Default output path: same folder and base name, with '_extracted.csv'
    if output_csv_path is None:
        output_csv_path = fit_path.with_name(f"{fit_path.stem}_extracted.csv")
    else:
        output_csv_path = Path(output_csv_path)
        
    print(f"\nProcessing: {fit_path.name} ...")
    
    try:
        fitfile = FitFile(str(fit_path))
    except Exception as e:
        print(f"[ERROR] Failed to load FIT file: {e}")
        return None

    raw_data_rows = []

    for record in fitfile.get_messages('record'):
        row_dict = {}
        
        # 1. Standard fields (timestamp, distance, etc.)
        for record_data in record:
            row_dict[record_data.name] = record_data.value
            
        # 2. Developer fields (acceleration HD, custom metrics)
        if hasattr(record, 'developer_fields'):
            for dev_data in record.developer_fields:
                row_dict[dev_data.name] = dev_data.value
                
        raw_data_rows.append(row_dict)

    if not raw_data_rows:
        print(f"[WARNING] No 'record' messages found in {fit_path.name}")
        return None

    df = pd.DataFrame(raw_data_rows)
    
    # Ensure parent output directory exists
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        df.to_csv(output_csv_path, index=False)
        print(f"[SUCCESS] Saved {len(df)} rows to: {output_csv_path.name}")
        print(f"Columns: {list(df.columns)}")
    except PermissionError:
        print(f"[ERROR] Permission denied for {output_csv_path.name}. Close Excel if the file is open!")
        
    return df


def batch_process_folder(folder_path: str | Path, recursive: bool = True):
    """
    Finds and converts all .fit / .FIT files in a directory.
    """
    folder = Path(folder_path)
    pattern = "**/*.[fF][iI][tT]" if recursive else "*.[fF][iI][tT]"
    fit_files = list(folder.glob(pattern))
    
    if not fit_files:
        print(f"No .FIT files found in {folder}")
        return
    
    print(f"Found {len(fit_files)} .FIT file(s). Starting extraction...")
    for fit_file in fit_files:
        parse_fit_to_csv(fit_file)
    print("\nAll conversions complete.")


# ==========================================
# USAGE EXAMPLES
# ==========================================
if __name__ == "__main__":
    
    # --- OPTION 1: Process a single file ---
    single_file = r"C:/"
    # parse_fit_to_csv(single_file)
    
    # --- OPTION 2: Batch process an entire trial folder ---
    # Automatically scans all subfolders for any .FIT file and outputs <filename>_extracted.csv
    trials_directory = r"C:\"
    batch_process_folder(trials_directory, recursive=True)
