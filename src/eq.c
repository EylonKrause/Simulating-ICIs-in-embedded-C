#include "eq.h"
#include "tx.h"
#include "fixed.h"

#include <math.h>
#include <string.h>

void eq_init(eq_t *e)
{
    memset(e, 0, sizeof(*e));
    e->w[FFE_CURSOR] = 1.0;          /* centre spike: pass-through to start */
}

void eq_refresh_taps(eq_t *e)
{
    for (unsigned i = 0; i < FFE_TAPS; ++i) {
        e->w[i] = (real_t)hal_read_signed(REG_FFE_TAP(i), TAP_APPLY_BITS) * TAP_CODE_SCALE;
    }
    for (unsigned i = 0; i < DFE_TAPS; ++i) {
        e->b[i] = (real_t)hal_read_signed(REG_DFE_TAP(i), TAP_APPLY_BITS) * TAP_CODE_SCALE;
    }
}

real_t eq_step(eq_t *e, real_t y_in, unsigned train_sym, unsigned *sym_out)
{
    /* shift the sample history */
    for (unsigned i = FFE_TAPS - 1u; i > 0u; --i) {
        e->x[i] = e->x[i - 1u];
    }
    e->x[0] = y_in;

    /* feed-forward */
    real_t y = 0.0;
    for (unsigned i = 0; i < FFE_TAPS; ++i) {
        y += e->w[i] * e->x[i];
    }
    /* decision feedback: subtract postcursor ISI reconstructed from decisions */
    for (unsigned i = 0; i < DFE_TAPS; ++i) {
        y -= e->b[i] * e->d[i];
    }

    const unsigned sym = pam4_slice(y);
    if (sym_out != NULL) {
        *sym_out = sym;
    }

    /* Error against the training symbol if we have one, else against our own
     * decision. Data-aided converges to the true MMSE solution; decision-
     * directed can lock onto a degenerate one when the eye starts closed,
     * which is exactly the situation during bring-up. Hence training first. */
    const real_t ref = (train_sym == 0xFFFFFFFFu) ? pam4_level(sym)
                                                  : pam4_level(train_sym);
    const real_t err = ref - y;

    /* --- what real hardware accumulates ---------------------------------
     * Sign-sign: one comparator per operand, an add or a subtract, no
     * multiplier. At 100 GBd with 12 taps a full multiplier per tap per
     * symbol is not affordable in area or power -- that, and not convergence
     * quality, is why sign-sign LMS is what silicon implements. */
    const int32_t se = (int32_t)sgn32((int32_t)(err * 32768.0));
    for (unsigned i = 0; i < FFE_TAPS; ++i) {
        e->grad[i] += se * (int32_t)sgn32((int32_t)(e->x[i] * 32768.0));
    }
    for (unsigned i = 0; i < DFE_TAPS; ++i) {
        e->dgrad[i] += se * (int32_t)sgn32((int32_t)(e->d[i] * 32768.0));
    }

    e->amp_acc += (int32_t)(fabs(y) * 4096.0);
    e->sym_cnt++;
    if (train_sym != 0xFFFFFFFFu && sym != train_sym) {
        e->err_cnt += pam4_bit_errors(sym, train_sym);
    }

    /* shift the decision history AFTER using it */
    for (unsigned i = DFE_TAPS - 1u; i > 0u; --i) {
        e->d[i] = e->d[i - 1u];
    }
    e->d[0] = pam4_level(sym);

    return y;
}

void eq_publish(const eq_t *e)
{
    for (unsigned i = 0; i < FFE_TAPS; ++i) {
        hw_reg_set(REG_GRAD_ACC(i), (uint32_t)e->grad[i]);
    }
    for (unsigned i = 0; i < DFE_TAPS; ++i) {
        hw_reg_set(REG_DFE_GRAD(i), (uint32_t)e->dgrad[i]);
    }
    hw_reg_set(REG_AMP_ACC, (uint32_t)e->amp_acc);
    hw_reg_set(REG_SYM_CNT, e->sym_cnt);
    hw_reg_set(REG_ERR_CNT, e->err_cnt);
}

void eq_clear_accumulators(eq_t *e)
{
    memset(e->grad, 0, sizeof(e->grad));
    memset(e->dgrad, 0, sizeof(e->dgrad));
    e->amp_acc = 0;
    e->sym_cnt = 0u;
    e->err_cnt = 0u;
}
