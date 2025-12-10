from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, List
from uuid import UUID
from sqlalchemy.orm import Session
from src.database import get_db
from src.services.simulation_service import SimulationService

router = APIRouter()

@router.get("/simulations")
async def get_available_simulations():
    """
    Get list of available simulation environments
    """
    simulation_service = SimulationService()
    simulations = simulation_service.get_available_simulations()
    return {"simulations": simulations}

@router.post("/simulations/{simulation_id}/launch")
async def launch_simulation(
    simulation_id: str,
    user_id: str,
    config: Dict[str, Any],
    db: Session = Depends(get_db)
):
    """
    Launch a simulation instance for a user
    """
    simulation_service = SimulationService()
    result = simulation_service.launch_simulation(simulation_id, user_id, config)

    if not result["success"]:
        raise HTTPException(status_code=400, detail=result.get("error", "Failed to launch simulation"))

    return result

@router.post("/simulations/{simulation_session_id}/stop")
async def stop_simulation(simulation_session_id: str):
    """
    Stop a running simulation instance
    """
    simulation_service = SimulationService()
    result = simulation_service.stop_simulation(simulation_session_id)

    if not result["success"]:
        raise HTTPException(status_code=400, detail="Failed to stop simulation")

    return result

@router.get("/simulations/{simulation_session_id}/status")
async def get_simulation_status(simulation_session_id: str):
    """
    Get the status of a running simulation
    """
    simulation_service = SimulationService()
    status = simulation_service.get_simulation_status(simulation_session_id)
    return status

@router.post("/simulations/{simulation_session_id}/execute-command")
async def execute_robot_command(
    simulation_session_id: str,
    command: str,
    params: Dict[str, Any]
):
    """
    Execute a robot command in the simulation
    """
    simulation_service = SimulationService()
    result = simulation_service.execute_robot_command(simulation_session_id, command, params)

    if not result["success"]:
        raise HTTPException(status_code=400, detail="Failed to execute command")

    return result