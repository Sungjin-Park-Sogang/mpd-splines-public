#!/usr/bin/env python3

"""
Test script for USD trajectory visualization with CuRobo.
This creates a few trajectories and saves them as USD files for Isaac Sim visualization.
"""

import sys
import os
from pathlib import Path

# Add the current directory to Python path
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

def test_usd_generation():
    """Test USD trajectory generation with a few sample trajectories."""
    print("=" * 60)
    print("Testing USD Trajectory Visualization")
    print("=" * 60)
    
    try:
        from generate_trajectories_curobo import GenerateDataCuRobo
        import torch
        
        print(f"CUDA available: {torch.cuda.is_available()}")
        
        # Create generator with USD saving enabled
        generator = GenerateDataCuRobo(
            env_id='EnvWarehouse',
            robot_id='RobotPanda',
            interpolate_num=128,
            save_usd=True,  # Enable USD saving
            usd_save_path="test_trajectory",  # File prefix
            debug=True
        )
        
        print(f"Created generator with USD saving enabled")
        print(f"USD files will be saved as: test_trajectory_XXXX.usd")
        
        # Generate a few trajectories
        num_test_trajectories = 3
        
        for i in range(num_test_trajectories):
            print(f"\n--- Generating trajectory {i+1}/{num_test_trajectories} ---")
            
            # Generate trajectory with random start/goal
            results_dict = generator.run(
                num_trajectories=1,
                max_tries=3,  # Allow a few retries
                joint_position_start=None,  # Random start
                joint_position_goal=None,   # Random goal
                planner_allowed_time=10.0,
                debug=True,
            )
            
            if results_dict and len(results_dict) > 0:
                result_key = list(results_dict.keys())[0]
                result = results_dict[result_key]
                
                if result.get('success', False):
                    sol_path = result['sol_path']
                    print(f"  ✓ Trajectory {i+1} generated successfully")
                    print(f"    Shape: {sol_path.shape}")
                    print(f"    Planning time: {result.get('solve_time', 'N/A'):.3f}s")
                    print(f"    USD file: test_trajectory_{i:04d}.usd")
                else:
                    print(f"  ✗ Trajectory {i+1} planning failed")
            else:
                print(f"  ✗ Trajectory {i+1} generation returned no results")
        
        # List generated USD files
        current_path = Path(".")
        usd_files = list(current_path.glob("test_trajectory_*.usd"))
        
        print(f"\n" + "=" * 60)
        print("USD FILES GENERATED")
        print("=" * 60)
        
        if usd_files:
            print(f"Found {len(usd_files)} USD files:")
            for usd_file in sorted(usd_files):
                file_size = usd_file.stat().st_size / 1024  # KB
                print(f"  📄 {usd_file.name} ({file_size:.1f} KB)")
            
            print(f"\n🎬 To visualize trajectories:")
            print(f"   1. Open Isaac Sim")
            print(f"   2. Open any of the generated .usd files")
            print(f"   3. Press Play button to see the robot motion animation")
            
        else:
            print("❌ No USD files were generated")
            return False
        
        return True
        
    except Exception as e:
        print(f"✗ USD generation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run USD visualization test."""
    print("USD Trajectory Visualization Test")
    print("This will generate a few trajectories and save them as USD files")
    print("for Isaac Sim visualization.\n")
    
    success = test_usd_generation()
    
    if success:
        print("\n🎉 USD visualization test completed successfully!")
        print("You can now open the generated .usd files in Isaac Sim to visualize the trajectories.")
    else:
        print("\n❌ USD visualization test failed.")
    
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)