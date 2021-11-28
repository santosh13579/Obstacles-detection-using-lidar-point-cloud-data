

**Chronic-Kidney-Diseases-Prediction* implements ML algorithm- Random Forest for classification. 

In this project:

- Identify risk factors for CKD 

- Predict potential CKD subtypes


## Installation Instruction

### Window 
1. conda create -n test python=3.6.7 pip
2. conda activate test
3. pip3 install -r requirements.txt (All reqiured packages are listed on requirements text file)
	```scikit-learn==0.23.0
	flask==1.0.2, scikit-learn==0.23.0,numpy==1.18.1, pandas==1.1.4, matplotlib==3.0.2...

4. go the project main folder (CKD_project) directory 
	```bash
	$> cd. ...\CKD_project_file\CKD_project
	```
5. train the model (train/test results will be saved in respective folder) 
	```bash
	$> ...\CKD_project_file\CKD_project>python train_model.py
	```
6. run python main.py 
	```bash
	$> ...\CKD_project_file\CKD_project>python main.py
	```
7. open  http://localhost:8080/
8. enter the feaures input on web API and press 'predict' button 

	
 ## Folder Structure Conventions

> Folder structure options and naming conventions for software projects

#### A top-level directory layout

    .
    ├── input                   # Compiled files (alternatively `dist`)
    ├── model                    # Documentation files (alternatively `doc`)
    ├── templets                     # Source files (alternatively `lib` or `app`)
    ├── utility_func                    # Automated tests (alternatively `spec` or `tests`)
    ├── CKD_train_model                  # Tools and utilities
    ├── main
    └── train_model
    
    

> Use short lowercase names at least for the top-level files and folders except
> `LICENSE`, `README.md`

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
