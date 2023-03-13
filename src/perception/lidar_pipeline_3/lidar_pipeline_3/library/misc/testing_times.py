import numpy
import perfplot


def stacking():
    perfplot.show(
        setup=lambda n: numpy.random.rand(n),
        kernels=[
            lambda a: numpy.c_[a, a],
            lambda a: numpy.stack([a, a]).T,
            lambda a: numpy.vstack([a, a]).T,
            lambda a: numpy.column_stack([a, a]),
            lambda a: numpy.concatenate([a[:, None], a[:, None]], axis=1),
        ],
        labels=["c_", "stack", "vstack", "column_stack", "concat"],
        n_range=[10**k for k in range(6)],
        xlabel="len(a)",
        logx=False,  # set to True or False to force scaling
        logy=False,
        target_time_per_measurement=2.0,
        time_unit="s",
    )


def masking():
    perfplot.show(
        setup=lambda n: numpy.random.rand(n),
        kernels=[
            lambda a: 0.5 * a,
            lambda a: a / 2,
        ],
        labels=["bool", "where", "masked_where"],
        n_range=[10**k for k in range(6)],
        xlabel="len(a)",
        logx=False,  # set to True or False to force scaling
        logy=False,
        target_time_per_measurement=2.0,
        time_unit="s",
    )


masking()
