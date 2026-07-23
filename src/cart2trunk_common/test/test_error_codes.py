from cart2trunk_common import error_codes


def test_all_codes_are_unique_nonempty_strings():
    values = [v for k, v in vars(error_codes).items() if not k.startswith('_') and isinstance(v, str)]
    assert len(values) > 0
    assert len(values) == len(set(values)), '중복된 error_code 상수가 있음'
    assert all(v == v.upper() and ' ' not in v for v in values)
