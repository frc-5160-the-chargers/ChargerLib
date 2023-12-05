package frc.chargers.utils


@DslMarker
public annotation class MappableContextMarker

/**
 * A context class used to create MutableMap's
 * using lambda blocks.
 */
@MappableContextMarker
public class MappableContext<K,V> {
    public var map: MutableMap<K,V> = mutableMapOf()

    public infix fun K.mapsTo(other: V){
        map[this] = other
    }
}