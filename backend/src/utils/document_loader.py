import os
from typing import Dict, List
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

def load_documents(source_path: str) -> Dict[str, str]:
    """
    Load markdown documents from the specified source path

    Args:
        source_path: Path to directory containing markdown files

    Returns:
        Dictionary mapping file paths to their content
    """
    documents = {}

    source_path_obj = Path(source_path)

    if not source_path_obj.exists():
        raise ValueError(f"Source path does not exist: {source_path}")

    if source_path_obj.is_file():
        # If it's a single file, load just that file
        if source_path_obj.suffix.lower() in ['.md', '.markdown']:
            content = _read_file(source_path)
            documents[str(source_path_obj)] = content
    else:
        # If it's a directory, load all markdown files in it and subdirectories
        for file_path in source_path_obj.rglob('*'):
            if file_path.is_file() and file_path.suffix.lower() in ['.md', '.markdown']:
                try:
                    content = _read_file(str(file_path))
                    documents[str(file_path)] = content
                    logger.info(f"Loaded document: {file_path}")
                except Exception as e:
                    logger.warning(f"Failed to load document {file_path}: {str(e)}")

    logger.info(f"Loaded {len(documents)} documents from {source_path}")
    return documents

def _read_file(file_path: str) -> str:
    """
    Read the content of a file

    Args:
        file_path: Path to the file to read

    Returns:
        File content as string
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()

def load_single_document(file_path: str) -> str:
    """
    Load a single document by file path

    Args:
        file_path: Path to the document file

    Returns:
        Document content as string
    """
    if not os.path.exists(file_path):
        raise ValueError(f"File does not exist: {file_path}")

    return _read_file(file_path)