# YouOnlyLiveOnce
### 역할 분담
- 인식: 안현석, 진창용
- 제어: 형승호, 장명근

### 데이터
left/right/stop/crosswalk/traffic_light_green_up/traffic_light_green_down/traffic_light_red_up/traffic_light_red_down/traffic_light_yellow
train [383, 150, 93, 100, 47, 50, 53, 77, 57, 48, 462, 597]  
eval [220, 26, 31, 32, 3, 3, 3, 4, 32, 0, 89, 345]  
뒤에 3개는 ignore 입니다.  

### 데이터 수집방법
자이카 카메라 활용

### Augmentation
- Resize 640x480 -> 416x146 (전이학습을 위한 resize)

### 학습 모델 포팅 여부
● onnx 변환  
○ trt 변환  
○ xycar환경 inference

### tensorboard
![Screenshot from 2023-01-19 18-51-42](https://user-images.githubusercontent.com/42567320/213411226-4e241cad-c571-4baa-bc8a-20ab2f431ff3.png)
![Screenshot from 2023-01-19 18-51-52](https://user-images.githubusercontent.com/42567320/213411245-d16be4d7-3dd6-467a-9e18-d9afec2ef95e.png)
![Screenshot from 2023-01-19 18-52-08](https://user-images.githubusercontent.com/42567320/213411261-8cad543a-5b72-4054-92f2-f95b4a63378d.png)

### 추가 적용 사항
- kmeans-clustering 구현 후 데이터에 적용하여 anchor box 추출

### 개선할 점
- 아직까지 쓸만한 학습결과가 나오지 않음

### 핑계
- 학습된 모델을 실수 날려 재학습 하는중입니다...
