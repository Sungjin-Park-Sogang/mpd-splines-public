#!/usr/bin/env python3

"""
Test script for fixed-length trajectory generation with CuRobo.
This tests the interpolate_trajectory_to_fixed_length functionality.
"""

import sys
import os
import numpy as np
import torch
from pathlib import Path

# Add the current directory to Python path
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

def test_interpolation_method():
    """Test the interpolate_trajectory_to_fixed_length method with mock data."""
    print("=" * 60)
    print("Testing interpolate_trajectory_to_fixed_length method")
    print("=" * 60)
    
    try:
        from generate_trajectories_curobo import GenerateDataCuRobo
        
        # Create generator with different interpolate_num values
        test_cases = [32, 64, 128, 256]
        
        for interpolate_num in test_cases:
            print(f"\nTesting with interpolate_num = {interpolate_num}")
            
            generator = GenerateDataCuRobo(
                env_id='EnvWarehouse',
                robot_id='RobotPanda', 
                interpolate_num=interpolate_num,
                debug=False
            )
            
            # Test with different original trajectory lengths
            original_lengths = [50, 100, 200, interpolate_num]  # Include same length case
            
            for orig_len in original_lengths:
                # Create mock trajectory data
                mock_trajectory = np.random.randn(orig_len, 7)  # orig_len waypoints, 7 DOF
                
                # Test interpolation
                interpolated = generator.interpolate_trajectory_to_fixed_length(mock_trajectory)
                
                expected_shape = (interpolate_num, 7)
                actual_shape = interpolated.shape
                
                print(f"  Original: {mock_trajectory.shape} -> Interpolated: {actual_shape}")
                
                # Verify shape
                assert actual_shape == expected_shape, f"Shape mismatch: expected {expected_shape}, got {actual_shape}"
                
                # Verify data type
                assert isinstance(interpolated, np.ndarray), f"Expected numpy array, got {type(interpolated)}"
                
                # Verify no NaN or infinite values
                assert not np.any(np.isnan(interpolated)), "Interpolated trajectory contains NaN values"
                assert not np.any(np.isinf(interpolated)), "Interpolated trajectory contains infinite values"
                
            print(f"  ✓ All tests passed for interpolate_num = {interpolate_num}")
        
        print("\n✓ All interpolation method tests passed!")
        return True
        
    except Exception as e:
        print(f"✗ Interpolation method test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_parameter_passing():
    """Test that interpolate_num parameter is properly passed through the system."""
    print("\n" + "=" * 60)
    print("Testing parameter passing")
    print("=" * 60)
    
    try:
        from generate_trajectories_curobo import GenerateDataCuRobo
        
        # Test different interpolate_num values
        test_values = [32, 64, 128, 256]
        
        for interpolate_num in test_values:
            generator = GenerateDataCuRobo(
                env_id='EnvWarehouse',
                robot_id='RobotPanda', 
                interpolate_num=interpolate_num,
                debug=False
            )
            
            # Verify the parameter was stored correctly
            assert hasattr(generator, 'interpolate_num'), "interpolate_num attribute not found"
            assert generator.interpolate_num == interpolate_num, f"interpolate_num mismatch: expected {interpolate_num}, got {generator.interpolate_num}"
            
            print(f"  ✓ interpolate_num = {interpolate_num} correctly stored")
        
        print("\n✓ All parameter passing tests passed!")
        return True
        
    except Exception as e:
        print(f"✗ Parameter passing test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_trajectory_generation_format():
    """Test that trajectory generation produces the expected output format."""
    print("\n" + "=" * 60)
    print("Testing trajectory generation output format")
    print("=" * 60)
    
    try:
        from generate_trajectories_curobo import GenerateDataCuRobo
        
        # Create generator with specific interpolate_num
        interpolate_num = 128
        generator = GenerateDataCuRobo(
            env_id='EnvWarehouse',
            robot_id='RobotPanda', 
            interpolate_num=interpolate_num,
            debug=True
        )
        
        print(f"Created generator with interpolate_num = {interpolate_num}")
        print(f"Generator has interpolate_num attribute: {hasattr(generator, 'interpolate_num')}")
        print(f"Generator interpolate_num value: {getattr(generator, 'interpolate_num', 'NOT_FOUND')}")
        
        # Test the run method with minimal parameters (dry run)
        print("\nTesting trajectory format (this may take a few seconds)...")
        
        # Generate one trajectory
        results_dict = generator.run(
            num_trajectories=1,
            max_tries=1,
            joint_position_start=None,  # Will use random
            joint_position_goal=None,   # Will use random
            planner_allowed_time=5.0,
            debug=True,
        )
        
        # Check results format
        if results_dict and len(results_dict) > 0:
            result_key = list(results_dict.keys())[0]
            result = results_dict[result_key]
            
            print(f"\nGenerated trajectory result:")
            print(f"  Success: {result.get('success', 'NOT_FOUND')}")
            
            if 'sol_path' in result:
                sol_path = result['sol_path']
                print(f"  sol_path shape: {sol_path.shape}")
                print(f"  Expected shape: ({interpolate_num}, 7)")
                print(f"  Shape match: {sol_path.shape == (interpolate_num, 7)}")
                
                # Verify interpolation_num field
                reported_interp_num = result.get('interpolation_num', 'NOT_FOUND')
                print(f"  Reported interpolation_num: {reported_interp_num}")
                print(f"  Matches expected: {reported_interp_num == interpolate_num}")
                
                if sol_path.shape == (interpolate_num, 7):
                    print("\n✓ Trajectory generation format test passed!")
                    return True
                else:
                    print(f"\n✗ Shape mismatch: expected ({interpolate_num}, 7), got {sol_path.shape}")
                    return False
            else:
                print("\n✗ sol_path not found in results")
                return False
        else:
            print("\n✗ No trajectory generated or results empty")
            return False
        
    except Exception as e:
        print(f"\n✗ Trajectory generation format test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_hdf5_compatibility():
    """Test that trajectories can be stored in HDF5 format without shape errors."""
    print("\n" + "=" * 60)
    print("Testing HDF5 storage compatibility")
    print("=" * 60)
    
    try:
        import h5py
        import tempfile
        
        # Create multiple mock trajectories with fixed length
        interpolate_num = 128
        num_trajectories = 5
        
        trajectories = []
        for i in range(num_trajectories):
            # All trajectories have the same shape
            traj = np.random.randn(interpolate_num, 7)
            trajectories.append(traj)
        
        print(f"Created {num_trajectories} trajectories with shape ({interpolate_num}, 7)")
        
        # Test HDF5 storage
        with tempfile.NamedTemporaryFile(suffix='.hdf5', delete=True) as tmp_file:
            with h5py.File(tmp_file.name, 'w') as hf:
                # Try to store trajectories as a dataset
                trajectories_array = np.array(trajectories)  # Shape: (5, 128, 7)
                
                hf.create_dataset('trajectories', data=trajectories_array)
                
                print(f"Successfully stored trajectories array with shape: {trajectories_array.shape}")
                print(f"HDF5 dataset shape: {hf['trajectories'].shape}")
                
                # Verify we can read it back
                loaded_trajectories = hf['trajectories'][:]
                print(f"Successfully loaded trajectories with shape: {loaded_trajectories.shape}")
                
                # Verify shapes match
                assert loaded_trajectories.shape == trajectories_array.shape
                
        print("\n✓ HDF5 storage compatibility test passed!")
        return True
        
    except Exception as e:
        print(f"\n✗ HDF5 storage compatibility test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("Fixed-Length Trajectory Generation Test Suite")
    print("=" * 60)
    
    print(f"Python version: {sys.version}")
    print(f"PyTorch available: {torch.__version__ if 'torch' in sys.modules else 'Not installed'}")
    print(f"CUDA available: {torch.cuda.is_available() if 'torch' in sys.modules else 'Unknown'}")
    
    # Run all tests
    tests = [
        test_interpolation_method,
        test_parameter_passing, 
        test_trajectory_generation_format,
        test_hdf5_compatibility,
    ]
    
    passed = 0
    failed = 0
    
    for test_func in tests:
        try:
            if test_func():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"Test {test_func.__name__} crashed: {e}")
            failed += 1
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Total:  {passed + failed}")
    
    if failed == 0:
        print("\n🎉 All tests passed! Fixed-length trajectory generation is working correctly.")
        return True
    else:
        print(f"\n❌ {failed} test(s) failed. Please check the implementation.")
        return False


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)