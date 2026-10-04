# 함께 수정하기

1. 이 저장소의 GitHub 페이지에서 **Fork**를 선택한다. 저장소 접근 권한과 GitHub의 포크 정책이 허용하는 계정에서 진행한다.
2. 본인 포크를 clone하고 `git switch -c feature/변경내용`으로 작업 브랜치를 만든다.
3. C 코드와 연결된 함수 명세·FSM 조건·기대 Command를 함께 확인한다. 테스트의 기대값을 구현 출력에 맞춰 무조건 바꾸지 않는다.
4. README의 빌드·테스트를 실행하고, 변경된 동작의 입력·기대값·실제값을 확인한다.
5. 변경 파일을 선택해 commit/push한 뒤 원본 저장소로 Pull Request를 연다. 변경 이유와 실행한 검사를 적는다.

원본 변경 사항을 가져올 때는 최초 한 번 upstream을 연결한다.

```sh
git remote add upstream https://github.com/KUCaver/rvc-controller.git
git fetch upstream
```

`upstream`이 이미 등록되어 있다면 다시 추가하지 않는다. GitHub에 표시되는 원본 기본 브랜치를 확인하고 작업 브랜치에 병합한다.

`build/`, `results/`, 원본 강의 자료, 개인 토큰·인증 파일을 commit하지 않는다. 기본 문서는 현재 구현 계약을 설명하며, 설계 해석 변경은 문서와 코드·테스트를 함께 검토한다.
