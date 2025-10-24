
import os
import io
import re
import shutil
import tempfile
import zipfile
from pathlib import Path
from typing import Optional, List
from uuid import uuid4

from fastapi import FastAPI, UploadFile, File, HTTPException, Form
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles


class ConversionServer:
    """
    Server-style API for converting a MuJoCo scene archive (zip) into a USDZ file.

    Contract:
    - Input: ZIP file containing one or more .xml files and their assets, and required xml_relpath form field to pick the XML inside the zip.
    - Output: JSON containing a relative download URL for the USDZ.
    - Error modes: 400 for bad input, 422 for missing/invalid XML, 500 on converter failures.
    """

    def __init__(self, base_dir: Optional[os.PathLike] = None):
        # All work happens under a temp base with a public folder for downloads
        self.base_dir = Path(base_dir) if base_dir else Path(tempfile.gettempdir()) / "mujoco_arviewer"
        self.jobs_dir = self.base_dir / "jobs"
        self.public_dir = self.base_dir / "public"
        self.public_dir.mkdir(parents=True, exist_ok=True)
        self.jobs_dir.mkdir(parents=True, exist_ok=True)

    # --- Core conversion primitive (reused from existing logic) ---
    def _convert_to_usdz(self, xml_path: str) -> str:
        """Convert MuJoCo XML to USDZ file and return USDZ path."""

        import mujoco_usd_converter, usdex.core
        from pxr import Usd, UsdUtils

        converter = mujoco_usd_converter.Converter()

        # Generate USDZ file path
        usd_output_path = xml_path.replace(".xml", "_usd")
        usdz_output_path = xml_path.replace(".xml", ".usdz")

        # Convert to USD first
        asset = converter.convert(xml_path, usd_output_path)
        stage = Usd.Stage.Open(asset.path)
        usdex.core.saveStage(stage, comment="modified after conversion")

        # Create USDZ package
        UsdUtils.CreateNewUsdzPackage(asset.path, usdz_output_path)
        print(f"✅ USDZ file created: {usdz_output_path}")

        return usdz_output_path

    # --- Helpers ---
    @staticmethod
    def _safe_extract_zip(zip_path: Path, extract_to: Path) -> None:
        """Safely extract zip to directory preventing path traversal (Zip Slip)."""
        extract_to = extract_to.resolve()
        with zipfile.ZipFile(zip_path) as zf:
            for member in zf.infolist():
                # Prevent absolute paths and parent traversal
                member_path = Path(member.filename)
                if member_path.is_absolute() or any(part == ".." for part in member_path.parts):
                    raise HTTPException(status_code=400, detail=f"Invalid file path in zip: {member.filename}")

                dest_path = (extract_to / member.filename).resolve()
                if not str(dest_path).startswith(str(extract_to)):
                    raise HTTPException(status_code=400, detail=f"Blocked unsafe path: {member.filename}")

                if member.is_dir():
                    dest_path.mkdir(parents=True, exist_ok=True)
                else:
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    with zf.open(member) as src, open(dest_path, "wb") as dst:
                        shutil.copyfileobj(src, dst)

    # Note: We used to guess candidate XMLs, but the API now requires xml_relpath.
    # The discovery helper was removed to avoid ambiguity and enforce explicitness.

    # --- High-level job orchestration ---
    def convert_zip_to_usdz(self, zip_path: Path, xml_relpath: str) -> Path:
        """
        Convert a zip archive with MuJoCo XML + assets to a USDZ file.
        Returns the final USDZ path in the public downloads directory.
        """
        job_id = uuid4().hex[:8]
        work_dir = self.jobs_dir / job_id
        work_dir.mkdir(parents=True, exist_ok=True)

        # Extract safely
        self._safe_extract_zip(Path(zip_path), work_dir)

        # Resolve XML path
        candidate = (work_dir / xml_relpath).resolve()
        if candidate.exists() and candidate.suffix.lower() == ".xml" and str(candidate).startswith(str(work_dir)):
            xml_path = candidate
        else:
            raise HTTPException(status_code=422, detail=f"Provided xml_relpath not found or invalid: {xml_relpath}")

        # Perform conversion in-place (USDZ will be next to XML)
        try:
            usdz_path = Path(self._convert_to_usdz(str(xml_path))).resolve()
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Conversion failed: {e}")

        # Move to public downloads with a stable name
        final_name = f"{xml_path.stem}-{job_id}.usdz"
        final_path = (self.public_dir / final_name).resolve()
        final_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(usdz_path), str(final_path))

        return final_path


# --- FastAPI wiring ---
server = ConversionServer()
app = FastAPI(title="MuJoCo → USDZ Converter API")


@app.post("/convert")
async def convert(
    file: UploadFile = File(..., description="ZIP containing MuJoCo XML and assets"),
    xml_relpath: str = Form(..., description="Relative path to the XML inside the zip (required)")
):
    if not file.filename or not file.filename.lower().endswith(".zip"):
        raise HTTPException(status_code=400, detail="Please upload a .zip file containing the MuJoCo XML and its assets.")

    # Persist upload to a temp file
    with tempfile.NamedTemporaryFile(delete=False, suffix=".zip") as tf:
        content = await file.read()
        if not content:
            raise HTTPException(status_code=400, detail="Uploaded file is empty.")
        tf.write(content)
        temp_zip_path = tf.name

    try:
        final_path = server.convert_zip_to_usdz(Path(temp_zip_path), xml_relpath=xml_relpath)
    finally:
        try:
            os.remove(temp_zip_path)
        except OSError:
            pass

    file_name = final_path.name
    return JSONResponse({
        "file_name": file_name,
        "download_url": f"/downloads/{file_name}",  # relative path; mount below
    })


# Serve the public download directory
app.mount("/downloads", StaticFiles(directory=str(server.public_dir)), name="downloads")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=int(os.getenv("PORT", "8000")),
        log_level="info",
    )

