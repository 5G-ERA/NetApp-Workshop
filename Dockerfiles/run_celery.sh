#!/bin/bash  
cd || exit
celery -A task worker -l info --pool=solo
