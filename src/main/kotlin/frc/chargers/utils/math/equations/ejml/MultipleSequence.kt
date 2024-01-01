package frc.chargers.utils.math.equations.ejml

import org.ejml.equation.Operation
import org.ejml.equation.Sequence

public operator fun Sequence.plus(other: Sequence): Sequence {
    return if (this is MultipleSequence) {
        addSequence(other)
        this
    } else {
        MultipleSequence(this, other)
    }
}

public class MultipleSequence(vararg sequences: Sequence) : Sequence() {
    private val sequences = mutableListOf(Sequence() /* Ensure at least one sequence in list */, *sequences)

    public fun addSequence(sequence: Sequence) {
        sequences.add(sequence)
    }

    override fun addOperation(operation: Operation?) {
        sequences.last().addOperation(operation)
    }

    override fun perform() {
        sequences.forEach { it.perform() }
    }
}