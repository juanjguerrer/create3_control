from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from create3_control.ros_actions import dock_robot, undock_robot, get_battery, DockingClient
import uvicorn
from fastapi.staticfiles import StaticFiles

app = FastAPI()
templates = Jinja2Templates(directory="/home/juanjg/ros2_ws/src/create3_control/templates")
app.mount("/static", StaticFiles(directory="/home/juanjg/ros2_ws/src/create3_control/static"), name="static")

@app.get("/", response_class=HTMLResponse)
async def homepage(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/dock")
async def dock():
    result = dock_robot()
    return {"status": result}

@app.post("/undock")
async def undock():
    result = undock_robot()
    return {"status": result}

@app.get("/status")
async def get_status():
    return {"battery": get_battery() }

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)