// Multi-Robot Sphero Controller - Frontend Application

class MultiRobotController {
    constructor() {
        this.spheros = [];
        this.currentSpheroToRemove = null;
        this.init();
    }

    init() {
        // Initialize event listeners
        this.setupEventListeners();

        // Load initial spheros
        this.loadSpheros();

        // Auto-refresh every 5 seconds
        setInterval(() => this.loadSpheros(), 5000);
    }

    setupEventListeners() {
        // Add Sphero button
        document.getElementById('addSpheroBtn').addEventListener('click', () => {
            this.showAddModal();
        });

        // Refresh button
        document.getElementById('refreshBtn').addEventListener('click', () => {
            this.loadSpheros();
        });

        // Add Sphero modal
        const addModal = document.getElementById('addSpheroModal');
        const confirmAddBtn = document.getElementById('confirmAddBtn');
        const cancelBtn = document.getElementById('cancelBtn');
        const spheroNameInput = document.getElementById('spheroNameInput');

        confirmAddBtn.addEventListener('click', () => {
            const spheroName = spheroNameInput.value.trim();
            if (spheroName) {
                this.addSphero(spheroName);
                this.hideAddModal();
            }
        });

        cancelBtn.addEventListener('click', () => {
            this.hideAddModal();
        });

        // Enter key in input
        spheroNameInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                confirmAddBtn.click();
            }
        });

        // Close modals on X click
        document.querySelectorAll('.close').forEach(closeBtn => {
            closeBtn.addEventListener('click', (e) => {
                const modal = e.target.closest('.modal');
                modal.classList.remove('show');
            });
        });

        // Close modals on outside click
        document.querySelectorAll('.modal').forEach(modal => {
            modal.addEventListener('click', (e) => {
                if (e.target === modal) {
                    modal.classList.remove('show');
                }
            });
        });

        // Confirm Remove modal
        const confirmRemoveBtn = document.getElementById('confirmRemoveBtn');
        const cancelRemoveBtn = document.getElementById('cancelRemoveBtn');

        confirmRemoveBtn.addEventListener('click', () => {
            if (this.currentSpheroToRemove) {
                this.removeSphero(this.currentSpheroToRemove);
                this.hideRemoveModal();
            }
        });

        cancelRemoveBtn.addEventListener('click', () => {
            this.hideRemoveModal();
        });
    }

    async loadSpheros() {
        try {
            const response = await fetch('/api/spheros');
            const data = await response.json();

            if (data.success) {
                this.spheros = data.spheros;
                this.renderSpheros();
                this.updateCount(data.count);
            }
        } catch (error) {
            console.error('Error loading spheros:', error);
        }
    }

    async addSphero(spheroName) {
        try {
            const response = await fetch('/api/spheros', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ sphero_name: spheroName })
            });

            const data = await response.json();

            if (data.success) {
                this.showNotification(`Sphero ${spheroName} added successfully!`, 'success');
                this.loadSpheros();
            } else {
                this.showNotification(`Failed to add Sphero: ${data.message}`, 'error');
            }
        } catch (error) {
            console.error('Error adding sphero:', error);
            this.showNotification('Error adding Sphero', 'error');
        }
    }

    async removeSphero(spheroName) {
        try {
            const response = await fetch(`/api/spheros/${spheroName}`, {
                method: 'DELETE'
            });

            const data = await response.json();

            if (data.success) {
                this.showNotification(`Sphero ${spheroName} removed successfully!`, 'success');
                this.loadSpheros();
            } else {
                this.showNotification(`Failed to remove Sphero: ${data.message}`, 'error');
            }
        } catch (error) {
            console.error('Error removing sphero:', error);
            this.showNotification('Error removing Sphero', 'error');
        }
    }

    renderSpheros() {
        const grid = document.getElementById('spheroGrid');
        const noSpherosMessage = document.getElementById('noSpherosMessage');

        if (this.spheros.length === 0) {
            grid.innerHTML = '';
            noSpherosMessage.classList.add('show');
        } else {
            noSpherosMessage.classList.remove('show');
            grid.innerHTML = this.spheros.map(sphero => this.createSpheroCard(sphero)).join('');

            // Add event listeners to cards
            this.spheros.forEach(sphero => {
                // Open button
                const openBtn = document.getElementById(`open-${sphero.name}`);
                if (openBtn) {
                    openBtn.addEventListener('click', () => {
                        window.open(sphero.url, '_blank');
                    });
                }

                // Remove button
                const removeBtn = document.getElementById(`remove-${sphero.name}`);
                if (removeBtn) {
                    removeBtn.addEventListener('click', () => {
                        this.showRemoveModal(sphero.name);
                    });
                }
            });
        }
    }

    createSpheroCard(sphero) {
        const statusClass = sphero.status || 'stopped';
        const statusText = statusClass.charAt(0).toUpperCase() + statusClass.slice(1);

        // Format timestamp
        const addedDate = new Date(sphero.added_at * 1000);
        const timeAgo = this.getTimeAgo(addedDate);

        return `
            <div class="sphero-card">
                <div class="card-header">
                    <div>
                        <h3 class="sphero-name">${sphero.name}</h3>
                    </div>
                    <div>
                        <span class="status-indicator ${statusClass}"></span>
                    </div>
                </div>

                <div class="card-info">
                    <div class="info-row">
                        <span class="info-label">Status:</span>
                        <span class="info-value">${statusText}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">WebSocket Port:</span>
                        <span class="info-value">${sphero.port}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Added:</span>
                        <span class="info-value">${timeAgo}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">URL:</span>
                        <span class="info-value" style="font-size: 0.9em;">${sphero.url}</span>
                    </div>
                </div>

                <div class="card-actions">
                    <button id="open-${sphero.name}" class="btn btn-primary">
                        <span class="icon">🎮</span> Open Controller
                    </button>
                    <button id="remove-${sphero.name}" class="btn btn-danger">
                        <span class="icon">🗑️</span> Remove
                    </button>
                </div>
            </div>
        `;
    }

    updateCount(count) {
        document.getElementById('spheroCount').textContent = count;
    }

    showAddModal() {
        const modal = document.getElementById('addSpheroModal');
        const input = document.getElementById('spheroNameInput');
        input.value = '';
        modal.classList.add('show');
        input.focus();
    }

    hideAddModal() {
        const modal = document.getElementById('addSpheroModal');
        modal.classList.remove('show');
    }

    showRemoveModal(spheroName) {
        this.currentSpheroToRemove = spheroName;
        const modal = document.getElementById('confirmRemoveModal');
        const text = document.getElementById('removeConfirmText');
        text.textContent = `Are you sure you want to remove Sphero "${spheroName}"? This will stop all controllers and close the WebSocket server.`;
        modal.classList.add('show');
    }

    hideRemoveModal() {
        const modal = document.getElementById('confirmRemoveModal');
        modal.classList.remove('show');
        this.currentSpheroToRemove = null;
    }

    showNotification(message, type = 'info') {
        // Create notification element
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.textContent = message;
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${type === 'success' ? '#4CAF50' : '#f44336'};
            color: white;
            padding: 16px 24px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
            z-index: 10000;
            animation: slideIn 0.3s ease;
        `;

        document.body.appendChild(notification);

        // Remove after 3 seconds
        setTimeout(() => {
            notification.style.animation = 'slideOut 0.3s ease';
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 300);
        }, 3000);
    }

    getTimeAgo(date) {
        const seconds = Math.floor((new Date() - date) / 1000);

        let interval = seconds / 31536000;
        if (interval > 1) return Math.floor(interval) + " years ago";

        interval = seconds / 2592000;
        if (interval > 1) return Math.floor(interval) + " months ago";

        interval = seconds / 86400;
        if (interval > 1) return Math.floor(interval) + " days ago";

        interval = seconds / 3600;
        if (interval > 1) return Math.floor(interval) + " hours ago";

        interval = seconds / 60;
        if (interval > 1) return Math.floor(interval) + " minutes ago";

        return Math.floor(seconds) + " seconds ago";
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    const controller = new MultiRobotController();
});

// Add CSS for notifications
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from {
            transform: translateX(400px);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }

    @keyframes slideOut {
        from {
            transform: translateX(0);
            opacity: 1;
        }
        to {
            transform: translateX(400px);
            opacity: 0;
        }
    }
`;
document.head.appendChild(style);
