import cymunk as cy
import pytest

def test_vec2d():
    v = cy.Vec2d(1, 2)
    assert(v is not None)
    assert(v.x == 1)
    assert(v.y == 2)
    assert(v[0] == 1)
    assert(v[1] == 2)

    v.x = 3
    v.y = 4
    assert(v.x == 3)
    assert(v.y == 4)
    assert(v[0] == 3)
    assert(v[1] == 4)

    v[0] = 5
    assert(v.x == 5)
    assert(v.y == 4)
    assert(v[0] == 5)
    assert(v[1] == 4)

    v[1] = 6
    assert(v.x == 5)
    assert(v.y == 6)
    assert(v[0] == 5)
    assert(v[1] == 6)
    
    assert v == (5,6)
    assert (5,6) == v
    assert v != (3,6)
    assert (3,6) != v


def test_vec2d_magic_math():
    import operator
    params = (
            (cy.Vec2d(2.1, 7), 3.2),
            (cy.Vec2d(2.3, 7), (3., 5.7)),
            (cy.Vec2d(-2.3, 7), cy.Vec2d(0.3, 5.7)),
        )
    ops = (
        operator.add, operator.sub, operator.mul,
        operator.div, operator.floordiv, operator.truediv,
        operator.mod,
        )
    for left, right in params:
        tright = right if hasattr(right, '__getitem__') else (right, right)
        for op in ops:
            # test op(left, right)
            msg = "%s( %s , %s )" % (op.__name__, left, right)
            tres = (op(left.x, tright[0]), op(left.y, tright[1]))
            res = op(left, right)
            assert tuple(res) == pytest.approx(tres), msg
            # test op(right, left)
            msg = "%s( %s , %s )" % (op.__name__, right, left)
            tres = (op(tright[0], left.x), op(tright[1], left.y))
            res = op(right, left)
            assert tuple(res) == pytest.approx(tres), msg


def test_vec2d_magic_math_inplace():
    import operator
    params = (
            (cy.Vec2d(2.1, 7), 3.2),
            (cy.Vec2d(2.3, 7), (3., 5.7)),
            (cy.Vec2d(-2.3, 7), cy.Vec2d(0.3, 5.7))
        )
    ops = (
        operator.iadd, operator.isub, operator.imul,
        operator.idiv, operator.ifloordiv, operator.itruediv,
        operator.imod
        )
    for left, right in params:
        tright = right if hasattr(right, '__getitem__') else (right, right)
        for op in ops:
            left = cy.Vec2d(left)
            tres = (op(left.x, tright[0]), op(left.y, tright[1]))
            op(left, right)
            msg = "%s( %s , %s )" % (op.__name__, left, right)
            assert tuple(left) == pytest.approx(tres), msg


def test_vec2d_magic_unary():
    import operator
    assert tuple(operator.neg(cy.Vec2d(-1,1))) == pytest.approx((1,-1))
    assert tuple(operator.pos(cy.Vec2d(-1,1))) == pytest.approx((-1,1))
    assert tuple(operator.abs(cy.Vec2d(-1,1))) == pytest.approx((1,1))


def test_vec2d_indexerror():
    v = cy.Vec2d(1, 2)
    with pytest.raises(IndexError):
        v[2] = 1
