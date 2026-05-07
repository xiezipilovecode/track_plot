@echo off
setlocal enabledelayedexpansion

echo Starting automated dataset collection...
echo.

call E:\programs\miniconda\Scripts\activate.bat
call conda activate carla
if errorlevel 1 (
    echo Conda activation failed
    pause
    exit /b 1
)

cd /d E:\code\track_plot

echo ============================================================
echo  Cleaning old dataset...
echo ============================================================
for /d %%d in (dataset\proxy_*) do (
    echo Removing %%d ...
    rmdir /s /q "%%d"
)
if exist dataset\coco_annotations.json    del dataset\coco_annotations.json
if exist dataset\batch_summary.json        del dataset\batch_summary.json
if exist dataset\auto_collect_config.json  del dataset\auto_collect_config.json
echo Done.

echo.
echo ============================================================
echo  Batch 1/6: medium density, medium speed (baseline)
echo ============================================================

set TP_PRESERVE_EXISTING_WORLD=1
set TT_SYNC_MODE=1
set TT_FIXED_DELTA_SECONDS=0.05

set TT_COLLECT_OUTPUT_DIR=dataset
set TT_COLLECT_FRAME_STRIDE=10
set TT_COLLECT_RECORD_LABELS=1
set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
set TT_COLLECT_WRITE_COCO=1
set TT_COLLECT_COCO_MIN_AREA_PX2=200
set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9

set TT_AUTO_COLLECT_ENABLE=1
set TT_AUTO_COLLECT_SECONDS_PER_RUN=180
set TT_AUTO_COLLECT_MAX_RUNS=50
set TT_AUTO_COLLECT_MODE=balanced
set TT_AUTO_COLLECT_TARGET_IMAGES_PER_CAMERA=50000
set TT_AUTO_COLLECT_COOLDOWN_S=5

set TT_SEED=100
set TT_PROXY_ENABLE=1
set TT_PROXY_USE_TM=0
set TT_PROXY_TARGET_PER_LANE=8
set TT_PROXY_BASE_SPEED_MPS=12
set TT_PROXY_FOLLOW_DISTANCE_M=10
set TT_PROXY_WARMUP_SECONDS=12

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Batch 2/6: high density, slow speed (congestion)
echo ============================================================

set TT_SEED=200
set TT_PROXY_TARGET_PER_LANE=12
set TT_PROXY_BASE_SPEED_MPS=8
set TT_PROXY_FOLLOW_DISTANCE_M=6
set TT_PROXY_WARMUP_SECONDS=15

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Batch 3/6: low density, high speed (cruising)
echo ============================================================

set TT_SEED=300
set TT_PROXY_TARGET_PER_LANE=4
set TT_PROXY_BASE_SPEED_MPS=15
set TT_PROXY_FOLLOW_DISTANCE_M=14
set TT_PROXY_WARMUP_SECONDS=8

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Batch 4/6: Traffic Manager mode
echo ============================================================

set TT_SEED=400
set TT_PROXY_USE_TM=1
set TT_TM_PORT=8000
set TT_TM_IGNORE_LIGHTS=1
set TT_TM_IGNORE_SIGNS=1
set TT_TM_AUTO_LANE_CHANGE=0
set TT_TM_FOLLOW_DISTANCE=8
set TT_TM_SPEED_DIFF_PERCENT=12
set TT_PROXY_TARGET_PER_LANE=8
set TT_PROXY_WARMUP_SECONDS=10

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Batch 5/6: normal mode refill
echo ============================================================

set TT_SEED=500
set TT_PROXY_USE_TM=0
set TT_PROXY_TARGET_PER_LANE=10
set TT_PROXY_BASE_SPEED_MPS=11
set TT_PROXY_FOLLOW_DISTANCE_M=9
set TT_PROXY_WARMUP_SECONDS=12

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Batch 6/6: top-up to target
echo ============================================================

set TT_SEED=600
set TT_PROXY_USE_TM=0
set TT_PROXY_TARGET_PER_LANE=9
set TT_PROXY_BASE_SPEED_MPS=13
set TT_PROXY_FOLLOW_DISTANCE_M=8
set TT_PROXY_WARMUP_SECONDS=10

python -m tp_tunnel_traffic.tests.test_auto_collect

echo.
echo ============================================================
echo  Merging COCO...
echo ============================================================
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset

echo.
echo ============================================================
echo  ALL DONE
echo ============================================================
echo Output: E:\code\track_plot\dataset\
echo COCO:   dataset\coco_annotations.json
echo Stats:  dataset\batch_summary.json
pause
