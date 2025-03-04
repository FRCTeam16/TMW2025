package frc.robot.util;

import java.util.Optional;

/**
 * A simple generic time-expiring value.
 */
public class TimeExpiringValue<V> {
    private final long expirationMillis;
    private V value;
    private long timestamp;

    public TimeExpiringValue(long expirationMillis) {
        this.expirationMillis = expirationMillis;
    }

    public void set(V value) {
        this.value = value;
        this.timestamp = System.currentTimeMillis();
    }

    public Optional<V> get() {
        if (isExpired()) {
            return Optional.ofNullable(value);
        } else {
            return Optional.empty();
        }
    }

    public boolean isExpired() {
        return System.currentTimeMillis() - timestamp >= expirationMillis;
    }

    public long getTimestamp() {
        return timestamp;
    }
}
