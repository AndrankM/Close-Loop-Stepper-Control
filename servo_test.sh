#!/bin/bash
echo "Moving servo 5 to +45 then -45 then 0..."
curl -s -X POST http://127.0.0.1:5000/servo/5/angle -H "Content-Type: application/json" -d '{"angle": 45}'; echo
sleep 1
curl -s -X POST http://127.0.0.1:5000/servo/5/angle -H "Content-Type: application/json" -d '{"angle": -45}'; echo
sleep 1
curl -s -X POST http://127.0.0.1:5000/servo/5/angle -H "Content-Type: application/json" -d '{"angle": 0}'; echo
echo "Moving servo 6 to +45 then -45 then 0..."
curl -s -X POST http://127.0.0.1:5000/servo/6/angle -H "Content-Type: application/json" -d '{"angle": 45}'; echo
sleep 1
curl -s -X POST http://127.0.0.1:5000/servo/6/angle -H "Content-Type: application/json" -d '{"angle": -45}'; echo
sleep 1
curl -s -X POST http://127.0.0.1:5000/servo/6/angle -H "Content-Type: application/json" -d '{"angle": 0}'; echo
echo DONE
