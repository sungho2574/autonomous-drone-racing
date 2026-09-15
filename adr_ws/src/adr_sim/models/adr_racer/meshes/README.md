# adr_racer meshes

- `1345_prop_ccw.stl`, `1345_prop_cw.stl` — PX4 `x500_base` 의 13.45" 프로펠러 메시 (BSD-3, `LICENSE.px4-gazebo-models`).
  `model.sdf` 에서 `<scale>` 로 5" 크기(5/13.45 ≈ 0.372)로 축소해 사용한다.
- 프레임 외관 STL 은 아직 없음 → `model.sdf` 의 `body_visual`(박스)이 placeholder.
  실제 STL 을 여기에 `frame.stl` 로 넣고 `scripts/gen_racer_model.py` 의 `body_visual` 을
  `<mesh><uri>model://adr_racer/meshes/frame.stl</uri><scale>…</scale></mesh>` 로 바꾼 뒤 재생성할 것.
  (STL 단위가 mm 이면 scale 0.001)
