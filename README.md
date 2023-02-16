# Programers-devcourse 자율주행
# - YOLO기반 표지판 검출 및 차선인식 주행 프로젝트  

## Team5 YouOnlyLiveOnce
## 프로젝트 rule
- 차선 유지 주행
- 갈림길 시 road sign(left or right) 판단 후 주행
- 정지선 + stop, crosswalk sign 시 5초 정지 후 출발
- 정지선 + red light 에서 정지 후 green light 변경 시 출발

## 팀원 역할 담당
- 안현석: YOLO 학습, 데이터 수집, Labeling
- 진창용: 데이터 수집, Labeling, 모델 변환
- 형승호: Hough transform기반 차선인식, 인식된 표지판을 사용하여 주행 전략 및 주행 제어 구현
- 장명근: Hough circle을 사용한 신호등 색 인식 

## 개발 환경
- xycar: Jetson tx2 보드 사용
- training: AWS server, 개인 PC
- 코딩 언어
  - YOLO: Python, Pytorch
  - 차선 인식, 주행 제어: ROS, C++

## YOLO 기반 표지판 검출
### YOLO
- Jetson tx2 보드 사용으로 YOLOv3-tiny 모델 사용
- server, pc에서 학습후 xycar에 모델 탑재
- pth file -> onnx -> trt
- 강의에서 제공되는 가이드라인 코드 기반 학습
- Darknet에서 제공되는 weight기반 전이학습

### 데이터셋
- Labeling class: Left, Right, Stop, Crosswalk, ~~Uturn~~, Traffic_light  
- train data objects: [767, 583, 271, 284, ~~23~~, 268]  
- eval data objects: [79, 46, 42, 75, ~~0~~, 62]   
- 알아볼 수 없거나, 특정 크기 이하의 object는 masking처리하여 사용  
- Uturn은 오검출을 방지하기위해 학습하였으며, 실제 환경에서는 미사용
- Left sign  
![left](https://user-images.githubusercontent.com/42567320/215160123-76c039d4-3ebb-41cb-a5da-c167fa74ff71.png)

- Right sign  
![right](https://user-images.githubusercontent.com/42567320/215160139-0e901690-72b8-41a5-8b06-e89b47acccc4.png)

- Stop sign  
![stop](https://user-images.githubusercontent.com/42567320/215160166-0289b56f-b245-4d65-a0d2-ade6162c8e47.png)

- Crosswalk sign  
![crosswalk](https://user-images.githubusercontent.com/42567320/215160219-b6f91f9b-95b8-4e37-b521-284584e2547c.png)

- ~~Uturn sign~~  
![uturn](https://user-images.githubusercontent.com/42567320/215160241-20824526-6971-417d-9cfb-fc4af4080829.png)

- Traffic light  
![trafic](https://user-images.githubusercontent.com/42567320/215160258-60707c09-f568-4c56-a6a9-d4e75a14a182.png)


### 데이터 수집 방법
- xycar에 장착된 카메라(ELP 2MP OV2710 CMOS) 사용
- 640 * 480 크기의 이미지를 학습에 사용
- "https://github.com/developer0hye/Yolo_Label"을 사용하여 라벨링 

### Augmentation  
**imgaug 라이브러리 활용**  
- Sharpen: 0.0 ~ 0.1  
- Affine: translate(-0.1 ~ 0.1), rotate(-3 ~ 3), scale(1.0, 1.5)  
- Trans_brightness: 0.8 ~ 0.95  
- Horizontal_flip  

### 추가 적용
- masking: 데이터중 필요없거나, 특정 크기 이하의 object masking  
- make_all: image, annotation file 확인 후 이상없거나, object존재 시 all.txt에 리스트로 작성   
- count_object: dataset중 object별 전체 수를 count  
- kmeans: masking 처리후 object의 크기를 기반으로 kmeans clustering기법을 통해 anchor 추출  
- image_show: labeling 결과 확인  


### 학습 결과
![evaluation](https://user-images.githubusercontent.com/42567320/215160737-7ac445b2-d397-4769-8892-e00396903fb9.png)
![tensorboard](https://user-images.githubusercontent.com/42567320/215160297-05a0b4e5-a69b-4125-8534-ca884e95c5f3.png)

## 제어 알고리즘
- flowchart 방식으로 표현 (todo)

## 신호등 색 분류 방법
### 신호등 분류 알고리즘
- 검출된 신호등 ROI의 mid, mid - offset, mid + offset의 3개의 colume의 sample 추출  
- 추출된 sample의 RGB값을 분석하여 각 sample이 대표하는 색을 확인하고 voting방식 판단  
- 결과 이미지 추가 (todo)

### 신호등 분류 알고리즘 (시행착오)
- 검출된 신호등 ROI를 hough circle을 통해 원검출 후 해당 원 내부의 색 판단  
  -> 원 검출의 비정확성과, 과도한 연산량으로 탈락 
- 검출된 신호등 ROI를 특정 threshold 값 이상만 masking하고 해당 픽셀의 위치를 평균내어 검출한 원의 RGB값 분석으로 색상판단  
  -> 요구 정확도는 나오나, 채택한 알고리즘 대비 연산량이 많아 탈락

## **✅ 어려웠던 부분 & 개선 방법**
## **✅ 잘 되었던 부분**

### 최종결과
 - gif로 만들어 추가 (todo)
