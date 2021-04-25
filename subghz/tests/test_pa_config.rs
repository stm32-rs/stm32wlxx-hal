use stm32wl_hal_subghz::PaSel;

#[test]
fn pa_sel_ord() {
    assert!(PaSel::Lp < PaSel::Hp);
    assert!(PaSel::Hp > PaSel::Lp);
}
