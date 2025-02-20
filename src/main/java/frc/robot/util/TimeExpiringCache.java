package frc.robot.util;


/**
 * A simple generic time-expiring cache.
 *
 * @param <K> key type.
 * @param <V> value type.
 */
public class TimeExpiringCache<K, V> {
    private final long expirationMillis;
    private final java.util.concurrent.ConcurrentHashMap<K, CacheEntry<V>> cache = new java.util.concurrent.ConcurrentHashMap<>();

    public TimeExpiringCache(long expirationMillis) {
        this.expirationMillis = expirationMillis;
    }

    public void put(K key, V value) {
        cache.put(key, new CacheEntry<>(value, System.currentTimeMillis()));
    }

    public V get(K key) {
        CacheEntry<V> entry = cache.get(key);
        if (entry != null) {
            if (System.currentTimeMillis() - entry.timestamp < expirationMillis) {
                return entry.value;
            } else {
                cache.remove(key);
            }
        }
        return null;
    }

    private record CacheEntry<V>(V value, long timestamp) {}
}
