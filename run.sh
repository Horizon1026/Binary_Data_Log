cd ./build
./test_binary_data_log
./test_csv_file_to_binlog ../example/imu_data.csv ../../Workspace/output/imu_data.binlog
./test_csv_file_to_binlog ../example/test_data.csv ../../Workspace/output/test_data.binlog
cd ..
