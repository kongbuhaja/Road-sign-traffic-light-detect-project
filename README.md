# 자율주행 데브코스 객체인식 프로젝트를 위한 C++ 주행 모듈 코드  
Perception에 집중할 수 있도록 일부, 좌회전 우회전을 위한 주행 모듈 코드를 제공합니다.  
팀마다 주행방식과 인지방식에 차이가 있을 수 있으므로, 나머지 모듈을 팀마다의 개성을 살려 개발하여야 합니다.  

### 좌회전 우회전 주행 원리
`ros::Rate`와 한쪽 차선을 반대 차선의 위치(+-470)로 보정하는 기술을 사용  
일정 시간동안 한쪽 차선만을 사용하여 주행시키도록 작성  
```
int sleep_rate = 12;    // 샘플링 시간 조절
int time = 3;           // 일정 시간 조절
int cnt = 0;            // 카운트 넘버 초기화

ros::Rate rate(sleep_rate);     // ros::rate 변수 생성
max_cnt = static_cast<float>(sleep_rate) * time;    // 반복할 최대 반복 수 설정

while (static_cast<float>(cnt) < max_cnt) {
    // ...

    if (direction == "left") {
            rpos = lpos + 470;
        }
        else if (direction == "right") {
            lpos = rpos - 470;
        }
    
    // drive topic Publish

    cnt++;
    rate.sleep();

```
### 주의 사항
1. 자이카 마다 샘플링 Hz가 다를 수 있으므로 sleep_rate와 time를 적절히 조절해 max_cnt를 튜닝하는 작업이 필요합니다.  


